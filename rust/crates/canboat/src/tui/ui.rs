// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! ratatui rendering + keyboard handling.
//!
//! Three screens, plus three modals:
//!
//! * [`Screen::Devices`] — top-level list of source addresses with
//!   manufacturer / model / PGN-count columns. `Enter` drills in;
//!   `q` quits.
//!
//! * [`Screen::DeviceDetail`] — every cached `(pgn, secondary)`
//!   tuple this device has produced, with the latest update time
//!   and per-tuple receive count. The PGN-List-Transmit / Receive
//!   pair from PGN 126464 is shown at the bottom; `i` sends an ISO
//!   Request for 126464 so the device tells us what it actually
//!   carries.
//!
//! * [`Screen::EntryDetail`] — the latest JSON line for the selected
//!   `(pgn, src, secondary)` tuple, pretty-printed.
//!
//! Modals (rendered on top of whichever screen is active, in priority
//! order — the highest-priority match wins for both rendering and
//! key handling):
//!
//! 1. **Error** — shown whenever `AppState::status.last_error` carries
//!    a string the user hasn't yet acknowledged. Any key dismisses it.
//! 2. **Connecting** — shown at startup until both connections settle
//!    (success or failure). Auto-dismisses on clean two-way success;
//!    any key dismisses it manually.
//! 3. **Override** — opened with `o` from `DeviceDetail` for the
//!    highlighted PGN. Accepts a transmission interval in ms; `0`
//!    means "stop transmitting". Available only in live mode; the
//!    server owns override persistence and application, so the modal
//!    just sends a `canboatPgnOverride` Set to the overrides control
//!    port. A silenced PGN stays visible on its device's list (with
//!    an OFF marker, sourced from the server's override report) so
//!    the user can later re-enable it. The dedicated **Overrides**
//!    view lists every override in place and lets the user change or
//!    delete them.

use std::io::Stdout;
use std::path::PathBuf;

use anyhow::Result;
use crossterm::event::{KeyCode, KeyEvent, KeyEventKind, KeyModifiers};
use ratatui::Terminal;
use ratatui::backend::CrosstermBackend;
use ratatui::layout::{Alignment, Constraint, Direction, Layout, Rect};
use ratatui::style::{Color, Modifier, Style};
use ratatui::text::{Line, Span};
use ratatui::widgets::{
    Block, BorderType, Borders, Clear, HighlightSpacing, List, ListItem, ListState, Paragraph, Wrap,
};

use crate::tui::client::SaveFormat;
use crate::tui::filebrowser::FileBrowser;
use crate::tui::iso;
use crate::tui::menu::{
    self, DESKTOP_BG, Menu, MenuAction, MenuBar, MenuEntry, MenuItem, PANEL_BG, PANEL_BORDER,
    PANEL_FG, PANEL_TITLE, SURFACE_ACCENT, SURFACE_BG, SURFACE_FG,
};
use crate::tui::state::{
    AppState, DeviceInfo, Entry, HistoryRecord, Mode, OverrideRow, PgnLoadRow, Progress,
};

/// "Stop transmitting" interval value (ms) for a PGN 126208 Request.
const INTERVAL_OFF: u32 = 0;

/// Per-PGN well-known transmit defaults, used purely as UI hints
/// when the user opens the override dialog. Picking the right default
/// is on the user — this is just a starting value.
fn default_interval_hint(pgn: u32) -> u32 {
    match pgn {
        127251 => 100,
        127257 | 127250 => 100,
        129025 | 129026 | 129029 => 250,
        130306 => 100,
        _ => 1000,
    }
}

#[derive(Clone)]
pub enum Screen {
    Devices,
    DeviceDetail {
        src: u8,
    },
    EntryDetail {
        src: u8,
        pgn: u32,
        secondary: Option<String>,
        /// Which observation in the entry's `history_indices` is
        /// currently being displayed. Defaults to the latest on
        /// drill-in; ← / → step backwards / forwards through past
        /// records. Clamped at draw time so the buffer being mutated
        /// underneath us (new records arriving in live mode) doesn't
        /// produce a stale index.
        history_pos: usize,
        /// Where to return on Esc/Backspace/h. `Screen::DeviceDetail`
        /// when the user drilled in from the per-device PGN list;
        /// `Screen::TimeView` when they drilled in from a timeline
        /// row. Preserving the origin means "back" behaves the way
        /// the user expects instead of always dumping them on the
        /// device tree.
        return_to: Box<Screen>,
    },
    /// Chronological list of every observed record. Toggled with
    /// `t` (time) / `d` (devices). Most useful in log-replay mode
    /// where the timeline is finite and the user wants to see the
    /// exact sequence a device booted / claimed / negotiated in,
    /// but it works in live mode too.
    TimeView,
    /// `top`-style PGN-load view — every PGN aggregated across sources,
    /// sorted busiest-first by message rate, with a bar-graph column.
    /// Opened with `p`.
    PgnTop,
    /// Per-device NMEA 0183 output filter (live mode + pipeline only).
    /// Lists every source producing 0183 with the sentences it emits;
    /// Space toggles a whole source or an individual sentence, sending
    /// a PGN 262657 Set to the pipeline. Fed by its Report frames.
    Nmea0183,
    /// PGN-rate overrides the server holds (live mode + pipeline only).
    /// Lists each `(src, pgn)` override with its interval; `e`/Enter
    /// edits the interval, `d`/Del removes it — both via the PGN 262658
    /// control channel. Fed by the server's Report frames.
    Overrides,
}

pub struct App {
    pub screen: Screen,
    pub devices_state: ListState,
    pub detail_state: ListState,
    /// Selection state for the `Nmea0183` filter screen.
    pub nmea0183_state: ListState,
    /// Selection state for the `Overrides` screen.
    pub overrides_state: ListState,
    /// Selection state for the `TimeView` screen — one row per
    /// observation in `AppState::history`.
    pub time_state: ListState,
    /// First visible TimeView row, kept in sync with `time_state` at
    /// render time. Tracked ourselves (rather than via `ListState`'s
    /// internal offset) so we can window the timeline and only build
    /// the on-screen rows for a huge capture.
    pub time_offset: usize,
    /// Selection state for the `PgnTop` load view — one row per PGN.
    pub pgn_top_state: ListState,
    /// The top menu bar (which pull-down is open + highlight).
    pub menu: MenuBar,
    /// Whether the Help / About modal is showing.
    pub help_visible: bool,
    /// Startup intro animation frame counter. `Some(n)` while the
    /// C64/Spectrum boot animation is playing; `None` once it finishes
    /// or the user skips it with a keystroke.
    pub splash: Option<u16>,
    /// Open File-menu I/O dialog (Connect / Save), if any.
    pub io_modal: Option<IoModal>,
    /// Open File ▸ Load / Save file browser, if any.
    pub file_browser: Option<FileBrowser>,
    /// Directory the browser was last in, remembered across opens so a
    /// subsequent Load / Save resumes where the user left off.
    pub browse_dir: Option<PathBuf>,
    /// A File-menu operation awaiting execution by the event loop.
    pub pending_command: Option<PendingCommand>,
    /// An override the user just deleted on the Overrides screen. The
    /// event loop drops it from `AppState::overrides` on the next frame
    /// (we only hold a shared borrow of the state in the key handler),
    /// so the row disappears immediately instead of waiting for the
    /// server's report to stop listing it.
    pub pending_override_forget: Option<(u8, u32)>,
    /// Vertical scroll offset for the pretty-printed JSON in the
    /// `EntryDetail` screen. Reset to 0 when drilling in.
    pub entry_scroll: u16,
    pub modal: Option<OverrideModal>,
    /// Last status / error toast (cleared on next keystroke).
    pub toast: Option<String>,
    pub should_quit: bool,
    /// True once the user has dismissed the startup "Connecting…"
    /// modal (or it auto-dismissed itself after both connections
    /// came up clean).
    pub connecting_dismissed: bool,
    /// The most recent error message the user has already
    /// acknowledged. Whenever `state.status.last_error` is `Some(e)`
    /// and `e != last_acknowledged_error`, a fatal-error modal is
    /// drawn over everything until the user presses a key.
    pub last_acknowledged_error: Option<String>,
    /// Active text-entry prompt for `/` (search) or `f` (PGN filter)
    /// on the TimeView screen. When `Some`, keystrokes go into the
    /// buffer; Enter applies, Esc cancels.
    pub text_prompt: Option<TextPrompt>,
    /// Active source-select modal for `s` on the TimeView screen —
    /// a checkbox list of currently-known sources, Space to toggle,
    /// Enter to apply.
    pub src_select: Option<SrcSelect>,
    /// Currently active substring search on TimeView (from `/`).
    /// Case-insensitive match against a canonical row string
    /// (`<timestamp> src <src> pgn <pgn> <description>`). `n` / `N`
    /// step through matches; `/` with empty input clears.
    pub search_query: Option<String>,
    /// Currently active PGN allowlist (from `f`). `None` = show all;
    /// `Some([])` behaves the same but is never produced (empty
    /// input clears back to `None`).
    pub filter_pgns: Option<Vec<u32>>,
    /// Currently active source allowlist (from `s`). Same
    /// conventions as `filter_pgns`.
    pub filter_srcs: Option<Vec<u8>>,
}

/// One-line text prompt shown at the bottom of the screen when the
/// user presses `/` or `f` on TimeView. `kind` picks what Enter
/// does with the buffered text.
pub struct TextPrompt {
    pub kind: TextPromptKind,
    pub buffer: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TextPromptKind {
    /// `/` — substring search across formatted row text.
    Search,
    /// `f` — comma-separated PGN allowlist. Empty clears.
    FilterPgns,
}

/// State for the interactive source-select modal (`s` on TimeView).
/// The list is snapshot at open time from `AppState::device_list()`;
/// cursor + selected set live here so the user can toggle multiple
/// sources before hitting Enter.
pub struct SrcSelect {
    pub sources: Vec<(u8, String)>,
    pub selected: std::collections::HashSet<u8>,
    pub cursor: usize,
}

/// Text-entry dialog for File ▸ Connect. (Load and Save use the file
/// browser instead of a text field.)
pub struct IoModal {
    pub input: String,
}

/// A File-menu operation the UI has requested but which the async event
/// loop must actually carry out (it owns the state handle, the writer,
/// and the connection task handles). The UI just records intent here;
/// `main`'s loop drains it each iteration.
pub enum PendingCommand {
    Connect {
        host: String,
        snapshot_port: u16,
        stream_port: u16,
    },
    Load {
        path: PathBuf,
    },
    Save {
        path: PathBuf,
        format: SaveFormat,
    },
}

pub struct OverrideModal {
    pub src: u8,
    pub pgn: u32,
    pub input: String,
    pub manufacturer_code: Option<u16>,
    pub industry_code: Option<u8>,
    /// PGN description captured at modal open time, for display.
    pub description: Option<String>,
}

impl App {
    pub fn new() -> Self {
        let mut devices_state = ListState::default();
        devices_state.select(Some(0));
        let mut detail_state = ListState::default();
        detail_state.select(Some(0));
        let mut time_state = ListState::default();
        time_state.select(Some(0));
        let mut nmea0183_state = ListState::default();
        nmea0183_state.select(Some(0));
        let mut overrides_state = ListState::default();
        overrides_state.select(Some(0));
        let mut pgn_top_state = ListState::default();
        pgn_top_state.select(Some(0));
        Self {
            screen: Screen::Devices,
            devices_state,
            detail_state,
            nmea0183_state,
            overrides_state,
            time_state,
            time_offset: 0,
            pgn_top_state,
            menu: MenuBar::new(),
            help_visible: false,
            splash: Some(0),
            io_modal: None,
            file_browser: None,
            browse_dir: None,
            pending_command: None,
            pending_override_forget: None,
            entry_scroll: 0,
            modal: None,
            toast: None,
            should_quit: false,
            connecting_dismissed: false,
            last_acknowledged_error: None,
            text_prompt: None,
            src_select: None,
            search_query: None,
            filter_pgns: None,
            filter_srcs: None,
        }
    }

    /// Whether a history record passes the currently-active TimeView
    /// filters (PGN + source, AND-combined; `None` = pass all).
    fn row_visible(&self, h: &HistoryRecord) -> bool {
        self.filter_pgns
            .as_ref()
            .is_none_or(|list| list.contains(&h.pgn))
            && self
                .filter_srcs
                .as_ref()
                .is_none_or(|list| list.contains(&h.src))
    }

    /// `true` when no TimeView filter is active — the common case, where
    /// the visible set is exactly `state.history` and we can index it
    /// directly instead of materialising an index list.
    fn no_history_filter(&self) -> bool {
        self.filter_pgns.is_none() && self.filter_srcs.is_none()
    }

    /// Row visibility filter for TimeView. Returns the indices into
    /// `state.history` of the records that pass the currently-active
    /// filters. One-shot callers (search, drill-in) use this; the hot
    /// paths (navigation, render) use [`Self::visible_history_len`] /
    /// [`Self::visible_window`] to avoid allocating a multi-million-
    /// element list on every keystroke of a large capture.
    pub fn visible_history_indices(&self, state: &AppState) -> Vec<usize> {
        if self.no_history_filter() {
            return (0..state.history.len()).collect();
        }
        state
            .history
            .iter()
            .enumerate()
            .filter(|(_, h)| self.row_visible(h))
            .map(|(i, _)| i)
            .collect()
    }

    /// Number of visible TimeView rows, without allocating. `O(1)` when
    /// unfiltered.
    pub fn visible_history_len(&self, state: &AppState) -> usize {
        if self.no_history_filter() {
            return state.history.len();
        }
        state.history.iter().filter(|h| self.row_visible(h)).count()
    }

    /// The `history` indices for a viewport window: up to `len` visible
    /// rows starting at the `offset`-th visible row. Only builds the
    /// window (not the whole list), so rendering a huge timeline costs
    /// the screen height, not the file size.
    fn visible_window(&self, state: &AppState, offset: usize, len: usize) -> Vec<usize> {
        if self.no_history_filter() {
            let end = (offset + len).min(state.history.len());
            return (offset..end).collect();
        }
        state
            .history
            .iter()
            .enumerate()
            .filter(|(_, h)| self.row_visible(h))
            .skip(offset)
            .take(len)
            .map(|(i, _)| i)
            .collect()
    }

    /// Composed row list for the DeviceDetail screen: live cached
    /// entries for `src`, plus a synthetic row for every silenced
    /// override on this `src` whose PGN isn't already represented
    /// by a live entry. The synthetic row has `count = 0` (which
    /// [`format_entry_row`] reads as "show OFF instead of every
    /// X ms") and carries the description we captured when the
    /// user set the override, so even an aged-out PGN stays
    /// visible and labelled.
    pub fn detail_rows(&self, src: u8, state: &AppState) -> Vec<Entry> {
        let mut rows: Vec<Entry> = state.entries_for_src(src).into_iter().cloned().collect();
        let mut present: std::collections::HashSet<u32> = rows.iter().map(|e| e.pgn).collect();
        for ov in state.override_rows() {
            if ov.interval_ms != INTERVAL_OFF || ov.src != src || present.contains(&ov.pgn) {
                continue;
            }
            rows.push(synthesize_silenced_entry(&ov));
            present.insert(ov.pgn);
        }
        // A device's PGN 126464 Transmit list names PGNs by *number* only,
        // but a proprietary number is shared across manufacturers — so each
        // is resolved to the variant matching this device's ISO-claim
        // manufacturer (see [`crate::tui::state::variant_for`]). Surface each
        // advertised variant we don't already show as a count==0 row so an
        // override can be built for it — with the *correct* manufacturer
        // header, without which the device ignores the 126208 request. Dedup
        // by (pgn, manufacturer) so a variant still appears when a different
        // manufacturer's definition of the same number is live.
        let device_mfr = device_manufacturer(src, state);
        let mut shown: std::collections::HashSet<(u32, Option<u16>)> = rows
            .iter()
            .map(|e| (e.pgn, proprietary_codes(e).0))
            .collect();
        for pgn in state.pgn_lists_for_src(src).tx {
            if let Some(v) = advertised_variant(pgn, device_mfr)
                && shown.insert((pgn, variant_manufacturer(v)))
            {
                rows.push(synthesize_advertised_entry(v, src));
            }
        }
        rows.sort_by(|a, b| {
            a.pgn
                .cmp(&b.pgn)
                .then_with(|| a.secondary.cmp(&b.secondary))
        });
        rows
    }

    /// Build the menu bar for the current context. Rebuilt every frame
    /// and on every keystroke (cheap) so item enable/disable state
    /// always matches the active screen / mode.
    pub fn build_menus(&self, state: &AppState) -> Vec<Menu> {
        let live = state.status.mode == Mode::Live;
        let on_device_detail = matches!(self.screen, Screen::DeviceDetail { .. });
        let on_timeview = matches!(self.screen, Screen::TimeView);
        let not_devices = !matches!(self.screen, Screen::Devices);
        vec![
            Menu {
                title: "File",
                hotkey: 'F',
                entries: vec![
                    // Connect / Load reset the whole model, so they're
                    // disabled while a save is streaming out of it.
                    MenuItem::maybe(
                        "Connect…",
                        Some('C'),
                        "",
                        MenuAction::FileConnect,
                        state.save_progress.is_none(),
                    ),
                    MenuItem::maybe(
                        "Load…",
                        Some('L'),
                        "",
                        MenuAction::FileLoad,
                        state.save_progress.is_none(),
                    ),
                    // Save needs data, a finished load (no half-read
                    // history), and no other save already running.
                    MenuItem::maybe(
                        "Save…",
                        Some('S'),
                        "",
                        MenuAction::FileSave,
                        !state.history.is_empty()
                            && state.status.snapshot_loaded
                            && state.save_progress.is_none(),
                    ),
                    MenuEntry::Separator,
                    MenuItem::new("Quit", Some('Q'), "q", MenuAction::Quit),
                ],
            },
            Menu {
                title: "View",
                hotkey: 'V',
                entries: vec![
                    MenuItem::new("Devices", Some('D'), "d", MenuAction::ViewDevices),
                    MenuItem::new("Timeline", Some('T'), "t", MenuAction::ViewTimeline),
                    MenuItem::new("PGN Load", Some('P'), "p", MenuAction::ViewPgnLoad),
                    MenuItem::maybe(
                        "NMEA 0183 Filter",
                        Some('N'),
                        "n",
                        MenuAction::ViewNmea0183,
                        live,
                    ),
                    MenuItem::maybe(
                        "PGN Overrides",
                        Some('O'),
                        "o",
                        MenuAction::ViewOverrides,
                        live,
                    ),
                    MenuEntry::Separator,
                    MenuItem::maybe("Back", Some('B'), "Esc", MenuAction::Back, not_devices),
                ],
            },
            Menu {
                title: "Device",
                hotkey: 'D',
                entries: vec![
                    MenuItem::maybe(
                        "Open Entry",
                        Some('O'),
                        "Enter",
                        MenuAction::OpenSelected,
                        on_device_detail,
                    ),
                    MenuItem::maybe(
                        "ISO Request 126464",
                        Some('I'),
                        "i",
                        MenuAction::IsoRequest126464,
                        on_device_detail && live,
                    ),
                    MenuItem::maybe(
                        "Override Interval…",
                        Some('V'),
                        "o",
                        MenuAction::OverrideInterval,
                        on_device_detail && live,
                    ),
                ],
            },
            Menu {
                title: "Search",
                hotkey: 'S',
                entries: vec![
                    MenuItem::maybe("Find…", Some('F'), "/", MenuAction::SearchFind, on_timeview),
                    MenuItem::maybe(
                        "Find Next",
                        Some('N'),
                        "n",
                        MenuAction::SearchNext,
                        on_timeview,
                    ),
                    MenuItem::maybe(
                        "Find Previous",
                        Some('P'),
                        "N",
                        MenuAction::SearchPrev,
                        on_timeview,
                    ),
                    MenuEntry::Separator,
                    MenuItem::maybe(
                        "Filter PGNs…",
                        Some('G'),
                        "f",
                        MenuAction::FilterPgns,
                        on_timeview,
                    ),
                    MenuItem::maybe(
                        "Filter Sources…",
                        Some('S'),
                        "s",
                        MenuAction::FilterSources,
                        on_timeview,
                    ),
                ],
            },
            Menu {
                title: "Help",
                hotkey: 'H',
                entries: vec![
                    MenuItem::new("Keys", Some('K'), "F1", MenuAction::Help),
                    MenuItem::new("About…", Some('A'), "", MenuAction::About),
                ],
            },
        ]
    }

    /// Route a keystroke while the menu bar is open. Left/Right switch
    /// menus, Up/Down move the highlight, Enter / hotkey fire, Esc
    /// closes.
    fn handle_menu_key(
        &mut self,
        key: KeyEvent,
        state: &AppState,
        writer: &crate::tui::client::Writer,
    ) {
        let menus = self.build_menus(state);
        let Some(open) = self.menu.open else {
            return;
        };
        match key.code {
            KeyCode::Esc => {
                self.menu.open = None;
            }
            KeyCode::Left | KeyCode::Char('h') => {
                let n = menus.len();
                let next = (open + n - 1) % n;
                self.menu.open = Some(next);
                self.menu.item = menus[next].first_item();
            }
            KeyCode::Right | KeyCode::Char('l') => {
                let n = menus.len();
                let next = (open + 1) % n;
                self.menu.open = Some(next);
                self.menu.item = menus[next].first_item();
            }
            KeyCode::Down | KeyCode::Char('j') => {
                self.menu.item = menus[open].step(self.menu.item, 1);
            }
            KeyCode::Up | KeyCode::Char('k') => {
                self.menu.item = menus[open].step(self.menu.item, -1);
            }
            KeyCode::Enter => {
                if let Some(menu::MenuEntry::Item(item)) = menus[open].entries.get(self.menu.item)
                    && item.enabled
                {
                    let action = item.action;
                    self.menu.open = None;
                    self.run_menu_action(action, state, writer);
                }
            }
            KeyCode::Char(c) => {
                // Fire the first enabled item whose hotkey matches.
                let lc = c.to_ascii_lowercase();
                let hit = menus[open].entries.iter().find_map(|e| match e {
                    menu::MenuEntry::Item(it)
                        if it.enabled && it.hotkey.map(|h| h.to_ascii_lowercase()) == Some(lc) =>
                    {
                        Some(it.action)
                    }
                    _ => None,
                });
                if let Some(action) = hit {
                    self.menu.open = None;
                    self.run_menu_action(action, state, writer);
                }
            }
            _ => {}
        }
    }

    /// Perform a menu action. Actions map onto the same operations the
    /// single-key accelerators trigger; several delegate to shared
    /// helpers so the two entry points can't drift.
    fn run_menu_action(
        &mut self,
        action: MenuAction,
        state: &AppState,
        writer: &crate::tui::client::Writer,
    ) {
        match action {
            MenuAction::Quit => self.should_quit = true,
            MenuAction::FileConnect => self.open_connect_modal(state),
            MenuAction::FileLoad => self.open_file_browser(),
            MenuAction::FileSave => self.open_save_browser(),
            MenuAction::ViewDevices => self.screen = Screen::Devices,
            MenuAction::ViewTimeline => self.screen = Screen::TimeView,
            MenuAction::ViewPgnLoad => {
                self.pgn_top_state.select(Some(0));
                self.screen = Screen::PgnTop;
            }
            MenuAction::ViewNmea0183 => {
                if state.status.mode == Mode::Live {
                    self.nmea0183_state.select(Some(0));
                    self.screen = Screen::Nmea0183;
                }
            }
            MenuAction::ViewOverrides => {
                if state.status.mode == Mode::Live {
                    self.overrides_state.select(Some(0));
                    self.screen = Screen::Overrides;
                }
            }
            MenuAction::Back => self.go_back(),
            MenuAction::OpenSelected => self.activate_selection(state, writer),
            MenuAction::IsoRequest126464 => {
                if let Screen::DeviceDetail { src } = self.screen
                    && state.status.mode == Mode::Live
                {
                    self.send_iso_126464(src, writer);
                }
            }
            MenuAction::OverrideInterval => {
                if let Screen::DeviceDetail { src } = self.screen
                    && state.status.mode == Mode::Live
                {
                    self.open_override_modal(src, state);
                }
            }
            MenuAction::SearchFind => {
                if matches!(self.screen, Screen::TimeView) {
                    self.text_prompt = Some(TextPrompt {
                        kind: TextPromptKind::Search,
                        buffer: self.search_query.clone().unwrap_or_default(),
                    });
                }
            }
            MenuAction::SearchNext => {
                if matches!(self.screen, Screen::TimeView) {
                    self.jump_search(state, 1);
                }
            }
            MenuAction::SearchPrev => {
                if matches!(self.screen, Screen::TimeView) {
                    self.jump_search(state, -1);
                }
            }
            MenuAction::FilterPgns => {
                if matches!(self.screen, Screen::TimeView) {
                    self.open_filter_pgns_prompt();
                }
            }
            MenuAction::FilterSources => {
                if matches!(self.screen, Screen::TimeView) {
                    self.open_src_select(state);
                }
            }
            MenuAction::Help | MenuAction::About => self.help_visible = true,
        }
    }

    /// Return to the previous screen — the shared "Back" behaviour used
    /// by both the Esc accelerator and the View ▸ Back menu item.
    fn go_back(&mut self) {
        self.screen = match &self.screen {
            Screen::EntryDetail { return_to, .. } => (**return_to).clone(),
            Screen::Devices => Screen::Devices,
            _ => Screen::Devices,
        };
    }

    /// Activate the current selection on the focused screen — the
    /// shared "Enter" behaviour, reused by the Device ▸ Open menu item.
    fn activate_selection(&mut self, state: &AppState, writer: &crate::tui::client::Writer) {
        match self.screen {
            Screen::Devices => {
                let devs = state.device_list();
                if let Some(d) = self.devices_state.selected().and_then(|i| devs.get(i)) {
                    self.screen = Screen::DeviceDetail { src: d.src };
                    self.detail_state.select(Some(0));
                }
            }
            Screen::DeviceDetail { src } => {
                let rows = self.detail_rows(src, state);
                if let Some(row) = self.detail_state.selected().and_then(|i| rows.get(i)) {
                    if row.count == 0 {
                        return;
                    }
                    self.entry_scroll = 0;
                    let history_pos = row.history_indices.len().saturating_sub(1);
                    self.screen = Screen::EntryDetail {
                        src,
                        pgn: row.pgn,
                        secondary: row.secondary.clone(),
                        history_pos,
                        return_to: Box::new(Screen::DeviceDetail { src }),
                    };
                }
            }
            Screen::TimeView => self.drill_into_time_selection(state),
            Screen::Nmea0183 => self.toggle_nmea0183_selection(state, writer),
            Screen::Overrides => self.open_selected_override_modal(state),
            Screen::EntryDetail { .. } | Screen::PgnTop => {}
        }
    }

    /// Open the interval modal for the highlighted row on the Overrides
    /// screen so the user can change it. The modal Enter path sends a
    /// `canboatPgnOverride` Set exactly like the DeviceDetail `o` path.
    fn open_selected_override_modal(&mut self, state: &AppState) {
        let rows = state.override_rows();
        if let Some(ov) = self.overrides_state.selected().and_then(|i| rows.get(i)) {
            let desc = (!ov.description.is_empty()).then(|| ov.description.clone());
            self.modal = Some(OverrideModal {
                src: ov.src,
                pgn: ov.pgn,
                input: ov.interval_ms.to_string(),
                manufacturer_code: ov.manufacturer_code,
                industry_code: ov.industry_code,
                description: desc,
            });
        }
    }

    /// Delete the highlighted override on the Overrides screen: send a
    /// `canboatPgnOverride` Delete to the server and optimistically drop
    /// the row locally (the next server Report is authoritative). The
    /// local removal is deferred to the event loop via
    /// [`App::pending_override_forget`] because we only hold a shared
    /// borrow of the state here.
    fn delete_selected_override(&mut self, state: &AppState, writer: &crate::tui::client::Writer) {
        let rows = state.override_rows();
        if let Some(ov) = self.overrides_state.selected().and_then(|i| rows.get(i)) {
            let (src, pgn) = (ov.src, ov.pgn);
            if writer.send(iso::override_delete(src, pgn)) {
                self.pending_override_forget = Some((src, pgn));
                self.toast = Some(format!("Deleted override → src {src} pgn {pgn}"));
            } else {
                self.toast = Some("Writer channel closed".into());
            }
        }
    }

    /// Send an ISO Request for PGN 126464 to `src` (Device ▸ ISO
    /// Request). Shared by the `i` accelerator.
    fn send_iso_126464(&mut self, src: u8, writer: &crate::tui::client::Writer) {
        let line = iso::iso_request(src, 126464);
        if writer.send(line) {
            self.toast = Some(format!("Sent ISO Request 126464 → src {src}"));
        } else {
            self.toast = Some("Writer channel closed".into());
        }
    }

    /// Open the PGN 126208 override dialog for the highlighted row on
    /// device `src` (Device ▸ Override Interval). Shared by the `o`
    /// accelerator.
    fn open_override_modal(&mut self, src: u8, state: &AppState) {
        let rows = self.detail_rows(src, state);
        if let Some(row) = self.detail_state.selected().and_then(|i| rows.get(i)) {
            // Live entries, silenced placeholders, and advertised (126464)
            // rows all carry the proprietary manufacturer / industry header
            // in their real or synthetic `line`, so one path reads the codes
            // for every kind of row. Getting the manufacturer right is what
            // makes the 126208 override target the correct proprietary PGN —
            // a wrong manufacturer is silently ignored by the device.
            let (mfr, ind) = proprietary_codes(row);
            let desc = (!row.description.is_empty()).then(|| row.description.clone());
            self.modal = Some(OverrideModal {
                src,
                pgn: row.pgn,
                input: default_interval_hint(row.pgn).to_string(),
                manufacturer_code: mfr,
                industry_code: ind,
                description: desc,
            });
        }
    }

    /// Open the File ▸ Connect dialog, pre-filling the endpoint from the
    /// current connection where it makes sense.
    fn open_connect_modal(&mut self, state: &AppState) {
        let input = match state.status.mode {
            Mode::Live => format!(
                "{} {} {}",
                state.status.host, state.status.snapshot_port, state.status.stream_port
            ),
            Mode::Log => "localhost 2597 2598".to_string(),
        };
        self.io_modal = Some(IoModal { input });
    }

    /// The directory a browser should open in: wherever the user last
    /// browsed, else the current working directory, else `/`.
    fn browse_start(&self) -> PathBuf {
        self.browse_dir
            .clone()
            .or_else(|| std::env::current_dir().ok())
            .unwrap_or_else(|| PathBuf::from("/"))
    }

    /// Open the File ▸ Load file browser, resuming the last-browsed
    /// directory.
    pub fn open_file_browser(&mut self) {
        self.file_browser = Some(FileBrowser::open(self.browse_start()));
    }

    /// Open the File ▸ Save browser (resuming the last-browsed
    /// directory): navigate, edit the filename, pick the format, Enter
    /// to write.
    fn open_save_browser(&mut self) {
        self.file_browser = Some(FileBrowser::open_save(
            self.browse_start(),
            "canboat-capture.json".to_string(),
            SaveFormat::Analysed,
        ));
    }

    /// Key routing for the File ▸ Load / Save browser.
    ///
    /// Load: ↑↓/jk move, Enter/→/l descends or picks a file (queues a
    /// Load), ←/h/Backspace ascends, Esc cancels.
    ///
    /// Save: ↑↓ move, → descends a dir (or adopts a file's name), ←
    /// ascends, Tab toggles format, typing edits the filename, Enter
    /// writes to `dir/filename`, Esc cancels. (No vim/backspace nav —
    /// those edit the filename.)
    fn handle_file_browser_key(&mut self, key: KeyEvent) {
        let mut result: Option<PendingCommand> = None;
        let mut toast: Option<String> = None;
        let mut close = false;
        let new_dir;
        {
            let Some(fb) = self.file_browser.as_mut() else {
                return;
            };
            if fb.save {
                match key.code {
                    KeyCode::Esc => close = true,
                    KeyCode::Down => fb.move_selection(1),
                    KeyCode::Up => fb.move_selection(-1),
                    KeyCode::Left => fb.go_parent(),
                    KeyCode::Right => {
                        // Descend into a dir, or adopt a highlighted
                        // file's name into the filename field (overwrite
                        // helper).
                        if let Some(path) = fb.activate()
                            && let Some(name) = path.file_name()
                        {
                            fb.filename = name.to_string_lossy().into_owned();
                        }
                    }
                    KeyCode::Tab | KeyCode::BackTab => {
                        fb.format = match fb.format {
                            SaveFormat::Analysed => SaveFormat::Raw,
                            SaveFormat::Raw => SaveFormat::Analysed,
                        };
                        fb.filename = swap_extension(&fb.filename, fb.format.extension());
                    }
                    KeyCode::Backspace => {
                        fb.filename.pop();
                    }
                    KeyCode::Enter => {
                        if fb.filename.trim().is_empty() {
                            toast = Some("Enter a file name".into());
                        } else {
                            result = Some(PendingCommand::Save {
                                path: fb.save_path(),
                                format: fb.format,
                            });
                        }
                    }
                    KeyCode::Char(c) => fb.filename.push(c),
                    _ => {}
                }
            } else {
                match key.code {
                    KeyCode::Esc => close = true,
                    KeyCode::Down | KeyCode::Char('j') => fb.move_selection(1),
                    KeyCode::Up | KeyCode::Char('k') => fb.move_selection(-1),
                    KeyCode::Left | KeyCode::Char('h') | KeyCode::Backspace => fb.go_parent(),
                    KeyCode::Enter | KeyCode::Right | KeyCode::Char('l') => {
                        if let Some(path) = fb.activate() {
                            result = Some(PendingCommand::Load { path });
                        }
                    }
                    _ => {}
                }
            }
            new_dir = fb.cwd.clone();
        }
        // Remember the directory we ended up in so the next open resumes
        // here (captured after any descend / parent navigation).
        self.browse_dir = Some(new_dir);
        if let Some(t) = toast {
            self.toast = Some(t);
        }
        if close || result.is_some() {
            self.file_browser = None;
        }
        if let Some(cmd) = result {
            self.pending_command = Some(cmd);
        }
    }

    /// Open the File ▸ Load browser — used at startup when the TUI was
    /// launched without a source.
    pub fn prompt_load(&mut self) {
        self.open_file_browser();
    }

    /// Key routing for the Connect dialog. Enter parses the input into a
    /// [`PendingCommand::Connect`] for the event loop to run; Esc cancels.
    fn handle_io_modal_key(&mut self, key: KeyEvent) {
        let Some(m) = self.io_modal.as_mut() else {
            return;
        };
        match key.code {
            KeyCode::Esc => self.io_modal = None,
            KeyCode::Backspace => {
                m.input.pop();
            }
            KeyCode::Enter => {
                let raw = m.input.trim().to_string();
                self.io_modal = None;
                if raw.is_empty() {
                    self.toast = Some("(cancelled — empty input)".into());
                    return;
                }
                // Accept "host", "host port port", or "host:port:port";
                // ports default to 2597 / 2598.
                let norm = raw.replace(':', " ");
                let mut it = norm.split_whitespace();
                let host = it.next().unwrap_or("localhost").to_string();
                let snapshot_port = it.next().and_then(|s| s.parse().ok()).unwrap_or(2597);
                let stream_port = it.next().and_then(|s| s.parse().ok()).unwrap_or(2598);
                self.pending_command = Some(PendingCommand::Connect {
                    host,
                    snapshot_port,
                    stream_port,
                });
            }
            KeyCode::Char(c) => {
                m.input.push(c);
            }
            _ => {}
        }
    }

    /// Reset navigation state after a Connect / Load swaps out the
    /// underlying bus model, so no stale cursor / filter points into the
    /// old data.
    pub fn reset_views(&mut self) {
        self.screen = Screen::Devices;
        self.devices_state.select(Some(0));
        self.detail_state.select(Some(0));
        self.time_state.select(Some(0));
        self.time_offset = 0;
        self.nmea0183_state.select(Some(0));
        self.pgn_top_state.select(Some(0));
        self.filter_pgns = None;
        self.filter_srcs = None;
        self.search_query = None;
        self.menu.open = None;
        self.text_prompt = None;
        self.src_select = None;
    }

    /// Open the `f` (PGN allowlist) prompt pre-filled from the active
    /// filter. Shared by the `f` accelerator and Search ▸ Filter PGNs.
    fn open_filter_pgns_prompt(&mut self) {
        let buffer = self
            .filter_pgns
            .as_ref()
            .map(|list| {
                list.iter()
                    .map(|p| p.to_string())
                    .collect::<Vec<_>>()
                    .join(",")
            })
            .unwrap_or_default();
        self.text_prompt = Some(TextPrompt {
            kind: TextPromptKind::FilterPgns,
            buffer,
        });
    }

    /// Open the `s` source-select checkbox modal. Shared by the `s`
    /// accelerator and Search ▸ Filter Sources.
    fn open_src_select(&mut self, state: &AppState) {
        let devs = state.device_list();
        let sources: Vec<(u8, String)> = devs
            .iter()
            .map(|d| {
                let label = if d.manufacturer.is_empty() && d.model.is_empty() {
                    format!("src {}", d.src)
                } else {
                    format!(
                        "src {:3}  {} {}",
                        d.src,
                        d.manufacturer.as_str(),
                        d.model.as_str()
                    )
                };
                (d.src, label)
            })
            .collect();
        let selected = self
            .filter_srcs
            .as_ref()
            .cloned()
            .map(|v| v.into_iter().collect())
            .unwrap_or_else(|| sources.iter().map(|(s, _)| *s).collect());
        self.src_select = Some(SrcSelect {
            sources,
            selected,
            cursor: 0,
        });
    }

    /// Drill into the EntryDetail for the highlighted TimeView row.
    /// Shared by the TimeView Enter accelerator and Device ▸ Open.
    fn drill_into_time_selection(&mut self, state: &AppState) {
        let visible = self.visible_history_indices(state);
        if let Some(target) = self
            .time_state
            .selected()
            .and_then(|i| visible.get(i).copied())
            && let Some(row) = state.history.get(target)
        {
            let key = (row.pgn, row.src, row.secondary.clone());
            if let Some(entry) = state.entries.get(&key) {
                let history_pos = entry
                    .history_indices
                    .iter()
                    .position(|i| *i == target)
                    .unwrap_or(entry.history_indices.len().saturating_sub(1));
                self.entry_scroll = 0;
                self.screen = Screen::EntryDetail {
                    src: row.src,
                    pgn: row.pgn,
                    secondary: row.secondary.clone(),
                    history_pos,
                    return_to: Box::new(Screen::TimeView),
                };
            }
        }
    }

    /// Toggle the mute state of the highlighted NMEA 0183 row. Shared
    /// by the Nmea0183 Space/Enter accelerator and Device ▸ Open.
    fn toggle_nmea0183_selection(&mut self, state: &AppState, writer: &crate::tui::client::Writer) {
        let rows = state.nmea0183_rows();
        if let Some(row) = self.nmea0183_state.selected().and_then(|i| rows.get(i)) {
            let sentence = row
                .sentence
                .as_deref()
                .unwrap_or(crate::tui::state::NMEA0183_ALL);
            let line = iso::nmea0183_filter_set(row.src, sentence, !row.muted);
            if writer.send(line) {
                let what = row.sentence.clone().unwrap_or_else(|| "all".into());
                self.toast = Some(format!(
                    "{} {what} on src {}",
                    if row.muted { "Unmuted" } else { "Muted" },
                    row.src
                ));
            } else {
                self.toast = Some("Writer channel closed".into());
            }
        }
    }

    pub fn handle_key(
        &mut self,
        key: KeyEvent,
        state: &AppState,
        writer: &crate::tui::client::Writer,
    ) {
        if !matches!(key.kind, KeyEventKind::Press | KeyEventKind::Repeat) {
            return;
        }
        // Ctrl-C always quits, even with a modal open.
        if key.modifiers.contains(KeyModifiers::CONTROL) && matches!(key.code, KeyCode::Char('c')) {
            self.should_quit = true;
            return;
        }
        // Any key skips the startup intro animation.
        if self.splash.is_some() {
            self.splash = None;
            return;
        }
        // The fatal-error modal trumps every other input — dismiss
        // it by acknowledging the current error string, then return.
        if has_unacknowledged_error(self, state) {
            self.last_acknowledged_error = state.status.last_error.clone();
            return;
        }
        // The connecting modal eats every other keypress so a user
        // tapping a key while reading "Connecting…" doesn't accidentally
        // start navigating the (still empty) device list.
        if !self.connecting_dismissed {
            self.connecting_dismissed = true;
            return;
        }
        // The Help / About overlay swallows the next key to dismiss.
        if self.help_visible {
            self.help_visible = false;
            return;
        }
        // While a pull-down is open it captures all navigation.
        if self.menu.is_open() {
            self.handle_menu_key(key, state, writer);
            return;
        }
        // Open dialogs capture ALL input while up — including F10 / Alt /
        // F1 — so the menu (or Help) can't open behind them.
        // File ▸ Load / Save browser captures navigation while open.
        if self.file_browser.is_some() {
            self.handle_file_browser_key(key);
            return;
        }
        // File ▸ Connect dialog captures input while open.
        if self.io_modal.is_some() {
            self.handle_io_modal_key(key);
            return;
        }
        if self.modal.is_some() {
            self.handle_modal_key(key, writer);
            return;
        }
        // Text prompt (/ or f) trumps every other key on TimeView.
        if self.text_prompt.is_some() {
            self.handle_text_prompt_key(key, state);
            return;
        }
        // Source-select modal (s) trumps everything else too.
        if self.src_select.is_some() {
            self.handle_src_select_key(key);
            return;
        }
        // F10 activates the menu bar; Alt+letter jumps straight to a
        // menu. Both work from any screen, over the top of the normal
        // accelerators (but not over an open dialog, handled above).
        if matches!(key.code, KeyCode::F(10)) {
            let menus = self.build_menus(state);
            self.menu.open = Some(0);
            self.menu.item = menus[0].first_item();
            return;
        }
        if key.modifiers.contains(KeyModifiers::ALT)
            && let KeyCode::Char(c) = key.code
        {
            let menus = self.build_menus(state);
            if let Some(i) = menus.iter().position(|m| m.hotkey.eq_ignore_ascii_case(&c)) {
                self.menu.open = Some(i);
                self.menu.item = menus[i].first_item();
            }
            return;
        }
        // F1 opens Help from anywhere.
        if matches!(key.code, KeyCode::F(1)) {
            self.help_visible = true;
            return;
        }
        self.toast = None;
        // Left/right on EntryDetail steps through past instances.
        // Handled *before* the shared-borrow match so we can grab a
        // `&mut history_pos` inside the Screen without conflicting
        // with the `&self.screen` borrow the main match takes.
        if let Screen::EntryDetail { history_pos, .. } = &mut self.screen {
            match key.code {
                KeyCode::Left | KeyCode::Char('H') => {
                    *history_pos = history_pos.saturating_sub(1);
                    self.entry_scroll = 0;
                    return;
                }
                KeyCode::Right | KeyCode::Char('L') => {
                    *history_pos = history_pos.saturating_add(1);
                    self.entry_scroll = 0;
                    return;
                }
                _ => {}
            }
        }
        match (&self.screen, key.code) {
            (_, KeyCode::Char('q')) => self.should_quit = true,
            // Global view toggles — `t` opens TimeView; `d` returns
            // to the device tree from anywhere. Both preserve the
            // per-screen ListState so the user's position isn't lost
            // when toggling back.
            (_, KeyCode::Char('t')) => self.screen = Screen::TimeView,
            (_, KeyCode::Char('p')) => {
                self.pgn_top_state.select(Some(0));
                self.screen = Screen::PgnTop;
            }
            (Screen::PgnTop, KeyCode::Down) | (Screen::PgnTop, KeyCode::Char('j')) => {
                navigate_list(&mut self.pgn_top_state, state.pgn_load_rows().len(), 1);
            }
            (Screen::PgnTop, KeyCode::Up) | (Screen::PgnTop, KeyCode::Char('k')) => {
                navigate_list(&mut self.pgn_top_state, state.pgn_load_rows().len(), -1);
            }
            (Screen::PgnTop, KeyCode::Enter) => {
                // Drill from a busy PGN into the timeline, pre-filtered
                // to just that PGN — the "click the hungry process"
                // gesture from top(1).
                let rows = state.pgn_load_rows();
                if let Some(r) = self.pgn_top_state.selected().and_then(|i| rows.get(i)) {
                    self.filter_pgns = Some(vec![r.pgn]);
                    self.time_state.select(Some(0));
                    self.screen = Screen::TimeView;
                }
            }
            (Screen::PgnTop, KeyCode::Char('d')) => self.screen = Screen::Devices,
            (Screen::PgnTop, KeyCode::Esc)
            | (Screen::PgnTop, KeyCode::Backspace)
            | (Screen::PgnTop, KeyCode::Char('h')) => {
                self.screen = Screen::Devices;
            }
            (Screen::TimeView, KeyCode::Char('d')) => self.screen = Screen::Devices,
            (Screen::TimeView, KeyCode::Down) | (Screen::TimeView, KeyCode::Char('j')) => {
                let n = self.visible_history_len(state);
                navigate_list(&mut self.time_state, n, 1);
            }
            (Screen::TimeView, KeyCode::Up) | (Screen::TimeView, KeyCode::Char('k')) => {
                let n = self.visible_history_len(state);
                navigate_list(&mut self.time_state, n, -1);
            }
            (Screen::TimeView, KeyCode::Char('/')) => {
                self.text_prompt = Some(TextPrompt {
                    kind: TextPromptKind::Search,
                    buffer: self.search_query.clone().unwrap_or_default(),
                });
            }
            (Screen::TimeView, KeyCode::Char('f')) => {
                let buffer = self
                    .filter_pgns
                    .as_ref()
                    .map(|list| {
                        list.iter()
                            .map(|p| p.to_string())
                            .collect::<Vec<_>>()
                            .join(",")
                    })
                    .unwrap_or_default();
                self.text_prompt = Some(TextPrompt {
                    kind: TextPromptKind::FilterPgns,
                    buffer,
                });
            }
            (Screen::TimeView, KeyCode::Char('s')) => {
                let devs = state.device_list();
                let sources: Vec<(u8, String)> = devs
                    .iter()
                    .map(|d| {
                        let label = if d.manufacturer.is_empty() && d.model.is_empty() {
                            format!("src {}", d.src)
                        } else {
                            format!(
                                "src {:3}  {} {}",
                                d.src,
                                d.manufacturer.as_str(),
                                d.model.as_str()
                            )
                        };
                        (d.src, label)
                    })
                    .collect();
                let selected = self
                    .filter_srcs
                    .as_ref()
                    .cloned()
                    .map(|v| v.into_iter().collect())
                    .unwrap_or_else(|| sources.iter().map(|(s, _)| *s).collect());
                self.src_select = Some(SrcSelect {
                    sources,
                    selected,
                    cursor: 0,
                });
            }
            (Screen::TimeView, KeyCode::Char('n')) => {
                self.jump_search(state, 1);
            }
            (Screen::TimeView, KeyCode::Char('N')) => {
                self.jump_search(state, -1);
            }
            (Screen::TimeView, KeyCode::Enter) => {
                // Translate the visible-row cursor to an actual
                // `state.history` index — the filter may hide most
                // of the raw list.
                let visible = self.visible_history_indices(state);
                if let Some(target) = self
                    .time_state
                    .selected()
                    .and_then(|i| visible.get(i).copied())
                    && let Some(row) = state.history.get(target)
                {
                    // Drilling in from TimeView opens EntryDetail for
                    // the record the cursor is on, positioned to
                    // exactly that observation — so you can Esc back
                    // and left/right through nearby instances.
                    let key = (row.pgn, row.src, row.secondary.clone());
                    if let Some(entry) = state.entries.get(&key) {
                        let history_pos = entry
                            .history_indices
                            .iter()
                            .position(|i| *i == target)
                            .unwrap_or(entry.history_indices.len().saturating_sub(1));
                        self.entry_scroll = 0;
                        self.screen = Screen::EntryDetail {
                            src: row.src,
                            pgn: row.pgn,
                            secondary: row.secondary.clone(),
                            history_pos,
                            return_to: Box::new(Screen::TimeView),
                        };
                    }
                }
            }
            (Screen::TimeView, KeyCode::Esc) => self.screen = Screen::Devices,
            (Screen::Devices, KeyCode::Down) | (Screen::Devices, KeyCode::Char('j')) => {
                navigate_list(&mut self.devices_state, state.device_list().len(), 1);
            }
            (Screen::Devices, KeyCode::Up) | (Screen::Devices, KeyCode::Char('k')) => {
                navigate_list(&mut self.devices_state, state.device_list().len(), -1);
            }
            (Screen::Devices, KeyCode::Enter) => {
                let devs = state.device_list();
                if let Some(d) = self.devices_state.selected().and_then(|i| devs.get(i)) {
                    self.screen = Screen::DeviceDetail { src: d.src };
                    self.detail_state.select(Some(0));
                }
            }
            // NMEA 0183 filter view — live pipeline only (needs the
            // 262657 control channel to be answered).
            (Screen::Devices, KeyCode::Char('n')) if state.status.mode == Mode::Live => {
                self.nmea0183_state.select(Some(0));
                self.screen = Screen::Nmea0183;
            }
            // PGN Overrides view — live pipeline only (needs the 262658
            // control channel to be answered).
            (Screen::Devices, KeyCode::Char('o')) if state.status.mode == Mode::Live => {
                self.overrides_state.select(Some(0));
                self.screen = Screen::Overrides;
            }
            (Screen::Overrides, KeyCode::Down) | (Screen::Overrides, KeyCode::Char('j')) => {
                navigate_list(&mut self.overrides_state, state.override_rows().len(), 1);
            }
            (Screen::Overrides, KeyCode::Up) | (Screen::Overrides, KeyCode::Char('k')) => {
                navigate_list(&mut self.overrides_state, state.override_rows().len(), -1);
            }
            (Screen::Overrides, KeyCode::Enter) | (Screen::Overrides, KeyCode::Char('e')) => {
                self.open_selected_override_modal(state);
            }
            (Screen::Overrides, KeyCode::Char('d')) | (Screen::Overrides, KeyCode::Delete) => {
                self.delete_selected_override(state, writer);
            }
            (Screen::Overrides, KeyCode::Esc)
            | (Screen::Overrides, KeyCode::Backspace)
            | (Screen::Overrides, KeyCode::Char('h')) => {
                self.screen = Screen::Devices;
            }
            (Screen::Nmea0183, KeyCode::Down) | (Screen::Nmea0183, KeyCode::Char('j')) => {
                navigate_list(&mut self.nmea0183_state, state.nmea0183_rows().len(), 1);
            }
            (Screen::Nmea0183, KeyCode::Up) | (Screen::Nmea0183, KeyCode::Char('k')) => {
                navigate_list(&mut self.nmea0183_state, state.nmea0183_rows().len(), -1);
            }
            (Screen::Nmea0183, KeyCode::Char(' ')) | (Screen::Nmea0183, KeyCode::Enter) => {
                let rows = state.nmea0183_rows();
                if let Some(row) = self.nmea0183_state.selected().and_then(|i| rows.get(i)) {
                    let sentence = row
                        .sentence
                        .as_deref()
                        .unwrap_or(crate::tui::state::NMEA0183_ALL);
                    let line = iso::nmea0183_filter_set(row.src, sentence, !row.muted);
                    if writer.send(line) {
                        let what = row.sentence.clone().unwrap_or_else(|| "all".into());
                        self.toast = Some(format!(
                            "{} {what} on src {}",
                            if row.muted { "Unmuted" } else { "Muted" },
                            row.src
                        ));
                    } else {
                        self.toast = Some("Writer channel closed".into());
                    }
                }
            }
            (Screen::Nmea0183, KeyCode::Esc)
            | (Screen::Nmea0183, KeyCode::Backspace)
            | (Screen::Nmea0183, KeyCode::Char('h'))
            | (Screen::Nmea0183, KeyCode::Char('d')) => {
                self.screen = Screen::Devices;
            }
            (Screen::DeviceDetail { src }, KeyCode::Down)
            | (Screen::DeviceDetail { src }, KeyCode::Char('j')) => {
                let n = self.detail_rows(*src, state).len();
                navigate_list(&mut self.detail_state, n, 1);
            }
            (Screen::DeviceDetail { src }, KeyCode::Up)
            | (Screen::DeviceDetail { src }, KeyCode::Char('k')) => {
                let n = self.detail_rows(*src, state).len();
                navigate_list(&mut self.detail_state, n, -1);
            }
            (Screen::DeviceDetail { src }, KeyCode::Enter) => {
                let rows = self.detail_rows(*src, state);
                if let Some(row) = self.detail_state.selected().and_then(|i| rows.get(i)) {
                    // Silenced rows (`count == 0`) have a Null `line`
                    // and no live data to show in EntryDetail; skip
                    // the drill-in so the user doesn't land on a
                    // pretty-printed `null`.
                    if row.count == 0 {
                        return;
                    }
                    self.entry_scroll = 0;
                    // Start at the most recent observation. Saturating
                    // is fine — a live entry has count >= 1, and we
                    // skipped count==0 rows above.
                    let history_pos = row.history_indices.len().saturating_sub(1);
                    let return_src = *src;
                    self.screen = Screen::EntryDetail {
                        src: return_src,
                        pgn: row.pgn,
                        secondary: row.secondary.clone(),
                        history_pos,
                        return_to: Box::new(Screen::DeviceDetail { src: return_src }),
                    };
                }
            }
            (Screen::DeviceDetail { src }, KeyCode::Char('i'))
                if state.status.mode == Mode::Live =>
            {
                // Ask the device to publish its PGN List (Transmit/Receive).
                let line = iso::iso_request(*src, 126464);
                if writer.send(line) {
                    self.toast = Some(format!("Sent ISO Request 126464 → src {src}"));
                } else {
                    self.toast = Some("Writer channel closed".into());
                }
            }
            (Screen::DeviceDetail { src }, KeyCode::Char('o'))
                if state.status.mode == Mode::Live =>
            {
                self.open_override_modal(*src, state);
            }
            (Screen::DeviceDetail { .. }, KeyCode::Esc)
            | (Screen::DeviceDetail { .. }, KeyCode::Backspace)
            | (Screen::DeviceDetail { .. }, KeyCode::Char('h')) => {
                self.screen = Screen::Devices;
            }
            (Screen::EntryDetail { .. }, KeyCode::Down)
            | (Screen::EntryDetail { .. }, KeyCode::Char('j')) => {
                self.entry_scroll = self.entry_scroll.saturating_add(1);
            }
            (Screen::EntryDetail { .. }, KeyCode::Up)
            | (Screen::EntryDetail { .. }, KeyCode::Char('k')) => {
                self.entry_scroll = self.entry_scroll.saturating_sub(1);
            }
            (Screen::EntryDetail { .. }, KeyCode::PageDown)
            | (Screen::EntryDetail { .. }, KeyCode::Char(' ')) => {
                self.entry_scroll = self.entry_scroll.saturating_add(10);
            }
            (Screen::EntryDetail { .. }, KeyCode::PageUp) => {
                self.entry_scroll = self.entry_scroll.saturating_sub(10);
            }
            (Screen::EntryDetail { .. }, KeyCode::Home)
            | (Screen::EntryDetail { .. }, KeyCode::Char('g')) => {
                self.entry_scroll = 0;
            }
            (Screen::EntryDetail { .. }, KeyCode::End)
            | (Screen::EntryDetail { .. }, KeyCode::Char('G')) => {
                self.entry_scroll = u16::MAX;
            }
            (Screen::EntryDetail { return_to, .. }, KeyCode::Esc)
            | (Screen::EntryDetail { return_to, .. }, KeyCode::Backspace)
            | (Screen::EntryDetail { return_to, .. }, KeyCode::Char('h')) => {
                // Restore whatever screen the user drilled in from —
                // DeviceDetail when coming via the per-device PGN
                // list, TimeView when coming via the timeline. The
                // `return_to` is captured at drill-in time so
                // subsequent state changes can't mislead us.
                self.screen = (**return_to).clone();
            }
            _ => {}
        }
    }

    /// Key routing when the `/` or `f` text prompt is active.
    /// Digits + letters + hyphen + comma go into the buffer; Enter
    /// commits (kind-specific interpretation), Esc cancels without
    /// touching the filter/search state.
    fn handle_text_prompt_key(&mut self, key: KeyEvent, state: &AppState) {
        let Some(prompt) = self.text_prompt.as_mut() else {
            return;
        };
        match key.code {
            KeyCode::Esc => {
                self.text_prompt = None;
            }
            KeyCode::Backspace => {
                prompt.buffer.pop();
            }
            KeyCode::Enter => {
                let raw = prompt.buffer.trim().to_string();
                let kind = prompt.kind;
                self.text_prompt = None;
                match kind {
                    TextPromptKind::Search => {
                        if raw.is_empty() {
                            self.search_query = None;
                        } else {
                            self.search_query = Some(raw);
                            self.jump_search(state, 0);
                        }
                    }
                    TextPromptKind::FilterPgns => {
                        if raw.is_empty() {
                            self.filter_pgns = None;
                        } else {
                            let list: Vec<u32> = raw
                                .split(|c: char| c == ',' || c.is_whitespace())
                                .filter(|t| !t.is_empty())
                                .filter_map(|t| t.parse().ok())
                                .collect();
                            self.filter_pgns = (!list.is_empty()).then_some(list);
                            self.time_state.select(Some(0));
                        }
                    }
                }
            }
            KeyCode::Char(c) => {
                prompt.buffer.push(c);
            }
            _ => {}
        }
    }

    /// Key routing for the source-select checkbox modal. `Space`
    /// toggles the current cursor's source; `a` / `A` toggles all;
    /// Enter commits; Esc cancels.
    fn handle_src_select_key(&mut self, key: KeyEvent) {
        let Some(sel) = self.src_select.as_mut() else {
            return;
        };
        let len = sel.sources.len();
        match key.code {
            KeyCode::Esc => {
                self.src_select = None;
            }
            KeyCode::Enter => {
                let selected = sel.selected.clone();
                let full = sel.sources.iter().all(|(s, _)| selected.contains(s));
                self.src_select = None;
                // "All sources selected" is the same as no filter —
                // canonicalise so the status bar doesn't lie.
                self.filter_srcs = if full || selected.is_empty() {
                    None
                } else {
                    let mut v: Vec<u8> = selected.into_iter().collect();
                    v.sort_unstable();
                    Some(v)
                };
                self.time_state.select(Some(0));
            }
            KeyCode::Down | KeyCode::Char('j') => {
                if len > 0 {
                    sel.cursor = (sel.cursor + 1) % len;
                }
            }
            KeyCode::Up | KeyCode::Char('k') => {
                if len > 0 {
                    sel.cursor = (sel.cursor + len - 1) % len;
                }
            }
            KeyCode::Char(' ') => {
                if let Some((src, _)) = sel.sources.get(sel.cursor)
                    && !sel.selected.insert(*src)
                {
                    sel.selected.remove(src);
                }
            }
            KeyCode::Char('a') | KeyCode::Char('A') => {
                if sel.selected.len() == sel.sources.len() {
                    sel.selected.clear();
                } else {
                    sel.selected = sel.sources.iter().map(|(s, _)| *s).collect();
                }
            }
            _ => {}
        }
    }

    /// Advance search cursor by `dir` (`+1` next, `-1` prev, `0`
    /// stay-or-find-first). Search matches against the same
    /// canonical row string [`format_time_row`] renders — case-
    /// insensitive substring match. No-op when there's no query.
    fn jump_search(&mut self, state: &AppState, dir: i32) {
        let Some(q_lower) = self.search_query.as_ref().map(|s| s.to_lowercase()) else {
            return;
        };
        let visible = self.visible_history_indices(state);
        if visible.is_empty() {
            return;
        }
        let cur = self.time_state.selected().unwrap_or(0);
        let matches: Vec<usize> = visible
            .iter()
            .enumerate()
            .filter(|(_, hi)| {
                state
                    .history
                    .get(**hi)
                    .is_some_and(|h| row_search_string(h).to_lowercase().contains(&q_lower))
            })
            .map(|(vpos, _)| vpos)
            .collect();
        if matches.is_empty() {
            self.toast = Some(format!(
                "no matches for \"{}\"",
                self.search_query.as_deref().unwrap_or("")
            ));
            return;
        }
        let target = match dir.cmp(&0) {
            std::cmp::Ordering::Equal => matches
                .iter()
                .copied()
                .find(|m| *m >= cur)
                .unwrap_or(matches[0]),
            std::cmp::Ordering::Greater => matches
                .iter()
                .copied()
                .find(|m| *m > cur)
                .unwrap_or(matches[0]),
            std::cmp::Ordering::Less => matches
                .iter()
                .copied()
                .rev()
                .find(|m| *m < cur)
                .unwrap_or(*matches.last().unwrap()),
        };
        self.time_state.select(Some(target));
    }

    fn handle_modal_key(&mut self, key: KeyEvent, writer: &crate::tui::client::Writer) {
        let Some(modal) = self.modal.as_mut() else {
            return;
        };
        match key.code {
            KeyCode::Esc => {
                self.modal = None;
            }
            KeyCode::Backspace => {
                modal.input.pop();
            }
            KeyCode::Char(c) if c.is_ascii_digit() => {
                modal.input.push(c);
            }
            KeyCode::Enter => {
                let interval_ms: u32 = match modal.input.parse() {
                    Ok(v) => v,
                    Err(_) => {
                        self.toast = Some("Enter an integer in milliseconds".into());
                        return;
                    }
                };
                // The server owns override persistence and application:
                // we just send a `canboatPgnOverride` Set on the
                // overrides control port, addressed by src. The server
                // resolves src→ISO NAME, persists, and emits the 126208
                // Request onto the bus. The Overrides view refreshes from
                // the server's Report frames.
                let line = iso::override_set(
                    modal.src,
                    modal.pgn,
                    interval_ms,
                    modal.manufacturer_code,
                    modal.industry_code,
                );
                if writer.send(line) {
                    self.toast = Some(format!(
                        "Sent override → src {} pgn {} = {} ms",
                        modal.src, modal.pgn, interval_ms
                    ));
                } else {
                    self.toast = Some("Writer channel closed".into());
                }
                self.modal = None;
            }
            _ => {}
        }
    }
}

/// Read manufacturer + industry codes off a cached proprietary-PGN
/// record, so the override dialog can pre-fill them. Returns
/// `(None, None)` for non-proprietary PGNs.
///
/// These stay string-keyed on purpose: `"Manufacturer Code"` /
/// `"Industry Code"` are the standard proprietary-header field names
/// shared by every PGN in the proprietary ranges, so this reads them
/// generically across all of them — there is no single per-PGN
/// `field::…` constant that would apply.
fn proprietary_codes(entry: &Entry) -> (Option<u16>, Option<u8>) {
    if !is_proprietary_pgn(entry.pgn) {
        return (None, None);
    }
    let mfr = entry
        .line
        .pointer("/fields/Manufacturer Code")
        .and_then(|v| {
            v.as_i64()
                .or_else(|| v.pointer("/value").and_then(|n| n.as_i64()))
        })
        .and_then(|n| u16::try_from(n).ok());
    let ind = entry
        .line
        .pointer("/fields/Industry Code")
        .and_then(|v| {
            v.as_i64()
                .or_else(|| v.pointer("/value").and_then(|n| n.as_i64()))
        })
        .and_then(|n| u8::try_from(n).ok());
    (mfr, ind)
}

fn is_proprietary_pgn(pgn: u32) -> bool {
    matches!(pgn, 0xEF00..=0xEFFF | 0xFF00..=0xFFFF | 0x1EF00..=0x1EFFF | 0x1FF00..=0x1FFFF)
}

fn navigate_list(state: &mut ListState, len: usize, delta: i32) {
    if len == 0 {
        state.select(None);
        return;
    }
    let cur = state.selected().unwrap_or(0) as i32;
    let next = (cur + delta).rem_euclid(len as i32) as usize;
    state.select(Some(next));
}

pub type Tty = Terminal<CrosstermBackend<Stdout>>;

pub fn draw(tty: &mut Tty, app: &mut App, state: &AppState) -> Result<()> {
    // Clamp the entry-detail history cursor + scroll offset to
    // whatever the visible entry currently holds. Both can race
    // against ingest (live mode: new observations grow the
    // history; log mode: it's frozen) so re-check every frame.
    if let Screen::EntryDetail {
        src,
        pgn,
        secondary,
        history_pos,
        ..
    } = &mut app.screen
    {
        let key = (*pgn, *src, secondary.clone());
        if let Some(entry) = state.entries.get(&key) {
            let max_pos = entry.history_indices.len().saturating_sub(1);
            if *history_pos > max_pos {
                *history_pos = max_pos;
            }
            let line = entry
                .history_indices
                .get(*history_pos)
                .and_then(|idx| state.history.get(*idx))
                .map(|h| &h.line)
                .unwrap_or(&entry.line);
            let pretty = serde_json::to_string_pretty(line).unwrap_or_else(|_| line.to_string());
            let lines = pretty.lines().count() as u16;
            let max_scroll = lines.saturating_sub(1);
            if app.entry_scroll > max_scroll {
                app.entry_scroll = max_scroll;
            }
        }
    }
    // Auto-dismiss the connecting modal.
    //
    // * Live mode: wait for both connections to come up clean.
    // * Log mode: the modal talks about "connecting" — irrelevant
    //   for a file. Dismiss immediately; the status bar already
    //   shows `loading… / loaded` as the file is ingested.
    if !app.connecting_dismissed
        && (state.status.mode == Mode::Log
            || (state.status.snapshot_loaded
                && state.status.stream_connected
                && state.status.last_error.is_none()))
    {
        app.connecting_dismissed = true;
    }
    tty.draw(|f| render(f, app, state))?;
    Ok(())
}

/// Paint one full frame. Split out from [`draw`] (which owns the
/// terminal handshake + pre-clamp) so a `TestBackend` can drive it in
/// unit tests.
fn render(f: &mut ratatui::Frame<'_>, app: &mut App, state: &AppState) {
    // Startup intro: play the C64/Spectrum boot animation over a black
    // screen, then hand off to the real UI. Advancing the frame here
    // (rather than in the event loop) keeps the animation tied to
    // actual repaints.
    if let Some(frame) = app.splash {
        let area = f.area();
        let verb = if state.status.mode == Mode::Live {
            "CONNECTING"
        } else {
            "LOADING"
        };
        draw_splash(f, area, frame, verb);
        app.splash = (frame + 1 < SPLASH_FRAMES).then_some(frame + 1);
        return;
    }
    {
        let area = f.area();
        // Paint the blue desktop behind everything, Turbo-Pascal style.
        f.render_widget(
            Block::default().style(Style::default().bg(DESKTOP_BG)),
            area,
        );
        let chunks = Layout::default()
            .direction(Direction::Vertical)
            .constraints([
                Constraint::Length(1), // menu bar
                Constraint::Length(1), // connection info strip
                Constraint::Min(1),    // content window
                Constraint::Length(1), // status / hint line
            ])
            .split(area);
        let menus = app.build_menus(state);
        menu::draw_bar(f, chunks[0], &menus, &app.menu);
        draw_info_bar(f, chunks[1], state);
        match &app.screen {
            Screen::Devices => draw_devices(f, chunks[2], app, state),
            Screen::DeviceDetail { src } => draw_device_detail(f, chunks[2], app, state, *src),
            Screen::EntryDetail {
                src,
                pgn,
                secondary,
                history_pos,
                ..
            } => {
                draw_entry_detail(
                    f,
                    chunks[2],
                    state,
                    *src,
                    *pgn,
                    secondary.as_deref(),
                    *history_pos,
                    app.entry_scroll,
                );
            }
            Screen::TimeView => draw_time_view(f, chunks[2], app, state),
            Screen::PgnTop => draw_pgn_top(f, chunks[2], app, state),
            Screen::Nmea0183 => draw_nmea0183(f, chunks[2], app, state),
            Screen::Overrides => draw_overrides(f, chunks[2], app, state),
        }
        // The bottom line: text prompt when `/` or `f` is active,
        // otherwise toast / error / per-screen hint.
        if let Some(prompt) = &app.text_prompt {
            draw_text_prompt(f, chunks[3], prompt);
        } else {
            draw_bottom_bar(f, chunks[3], app, state);
        }
        // The open pull-down floats over the content but under the
        // dialog modals.
        menu::draw_dropdown(f, chunks[0], area, &menus, &app.menu);
        // Modal stack — last drawn wins. Override dialog is least
        // important, connecting overlay sits above it, fatal-error
        // modal trumps everything.
        if let Some(modal) = &app.modal {
            draw_modal(f, area, modal);
        }
        if let Some(io) = &app.io_modal {
            draw_io_modal(f, area, io);
        }
        if let Some(fb) = &app.file_browser {
            draw_file_browser(f, area, fb);
        }
        if let Some(sel) = &app.src_select {
            draw_src_select_modal(f, area, sel);
        }
        if app.help_visible {
            draw_help_modal(f, area);
        }
        // A running capture save floats above the dialogs (it starts
        // after they close) but under the connecting / error overlays.
        if let Some(p) = &state.save_progress {
            draw_progress_modal(f, area, p);
        }
        if !app.connecting_dismissed {
            draw_connecting_modal(f, area, state);
        }
        if has_unacknowledged_error(app, state)
            && let Some(err) = state.status.last_error.as_deref()
        {
            draw_error_modal(f, area, err);
        }
    }
}

/// True when there's an error in `state.status.last_error` that the
/// user hasn't yet dismissed via a keystroke.
fn has_unacknowledged_error(app: &App, state: &AppState) -> bool {
    match (
        state.status.last_error.as_deref(),
        app.last_acknowledged_error.as_deref(),
    ) {
        (Some(cur), Some(ack)) => cur != ack,
        (Some(_), None) => true,
        _ => false,
    }
}

/// Thin blue info strip under the menu bar: endpoint / connection /
/// counters. The action feedback (toast / error) now lives on the
/// bottom status line instead.
fn draw_info_bar(f: &mut ratatui::Frame<'_>, area: Rect, state: &AppState) {
    let s = &state.status;
    let text = match s.mode {
        Mode::Live => {
            let conn = if s.stream_connected { "live" } else { "disc" };
            let snap = if s.snapshot_loaded { "ok" } else { "…" };
            format!(
                " endpoint {host}:{snap_port}/{stream_port}  snap:{snap}  stream:{conn}  msgs:{msgs}  devs:{devs}  entries:{entries}",
                host = s.host,
                snap_port = s.snapshot_port,
                stream_port = s.stream_port,
                msgs = s.messages_seen,
                devs = state.device_list().len(),
                entries = state.entries.len(),
            )
        }
        Mode::Log => {
            let load = if s.snapshot_loaded {
                "loaded"
            } else {
                "loading…"
            };
            format!(
                " log: {host}  {load}  msgs:{msgs}  devs:{devs}  entries:{entries}",
                host = s.host,
                msgs = s.messages_seen,
                devs = state.device_list().len(),
                entries = state.entries.len(),
            )
        }
    };
    let p = Paragraph::new(text).style(Style::default().bg(DESKTOP_BG).fg(Color::White));
    f.render_widget(p, area);
}

/// Bottom status line (grey surface). Priority: toast > error >
/// per-screen hint.
///
/// * `toast` is a confirmation / hint for an action the user just took.
/// * `error` is a persistent connection / I/O error not yet dismissed.
/// * otherwise the per-screen keybinding cheat sheet.
fn draw_bottom_bar(f: &mut ratatui::Frame<'_>, area: Rect, app: &App, state: &AppState) {
    let surface = Style::default().bg(SURFACE_BG).fg(SURFACE_FG);
    f.render_widget(Block::default().style(surface), area);
    let line: Line<'static> = match (&app.toast, &state.status.last_error) {
        (Some(t), _) => Line::from(Span::styled(format!(" {t}"), surface)),
        (_, Some(e)) => Line::from(vec![
            Span::styled(
                " error: ",
                surface.fg(Color::Red).add_modifier(Modifier::BOLD),
            ),
            Span::styled(e.clone(), surface.fg(Color::Red)),
        ]),
        _ => Line::from(Span::styled(
            format!(" {}", screen_hint(&app.screen, state.status.mode)),
            surface,
        )),
    };
    f.render_widget(Paragraph::new(line).style(surface), area);
}

/// Per-screen one-line keybinding cheat sheet. Kept in one place so
/// the top status-bar fallback line and the bottom hint bar stay in
/// sync, and so a binding that doesn't apply (e.g. `i`/`o` on the
/// EntryDetail screen, or on the DeviceDetail screen in log mode)
/// doesn't get advertised where it would do nothing.
fn screen_hint(screen: &Screen, mode: Mode) -> &'static str {
    match (screen, mode) {
        (Screen::Devices, Mode::Live) => {
            "F10 menu | ↑↓ move | Enter open | n NMEA0183 | o overrides | t timeline | p PGN load | q quit"
        }
        (Screen::Devices, _) => {
            "F10 menu | ↑↓ move | Enter open | t timeline | p PGN load | q quit"
        }
        (Screen::Nmea0183, _) => {
            "F10 menu | ↑↓ move | Space toggle mute | d/Esc back | (row = whole source or one sentence)"
        }
        (Screen::Overrides, _) => {
            "F10 menu | ↑↓ move | e/Enter change interval | d/Del delete | Esc back"
        }
        (Screen::PgnTop, _) => {
            "F10 menu | ↑↓ move | Enter timeline for PGN | t timeline | d/Esc back | q quit"
        }
        // Log mode: no live bus → no `i` / `o`.
        (Screen::DeviceDetail { .. }, Mode::Log) => {
            "F10 menu | ↑↓ move | Enter open entry | Esc back | t timeline | p PGN load"
        }
        (Screen::DeviceDetail { .. }, Mode::Live) => {
            "F10 menu | ↑↓ move | Enter open | Esc back | i ISO 126464 | o override | p PGN load"
        }
        (Screen::EntryDetail { .. }, _) => {
            "F10 menu | ↑↓ scroll 1 | ←→ prev/next instance | PgUp/PgDn scroll 10 | Esc back"
        }
        (Screen::TimeView, _) => {
            "F10 menu | ↑↓ move | Enter drill in | / search | n/N next/prev | f filter pgns | s filter srcs | d devices | Esc back"
        }
    }
}

fn draw_devices(f: &mut ratatui::Frame<'_>, area: Rect, app: &mut App, state: &AppState) {
    let devices = state.device_list();
    let items: Vec<ListItem> = devices
        .iter()
        .map(|d| ListItem::new(format_device_row(d)))
        .collect();
    // Force-select row 0 the moment the list is non-empty (and the
    // user hasn't picked something else yet). Combined with
    // `HighlightSpacing::Always` below this keeps the column layout
    // stable from the first frame: the marker slot is reserved and
    // row 0 already wears the highlight, so the first ↓ keystroke
    // moves cursor → row 1 without shifting any column to the right.
    if !devices.is_empty() && app.devices_state.selected().is_none() {
        app.devices_state.select(Some(0));
    }
    let list = List::new(items)
        .block(panel_block(format!(" Devices ({} src) ", devices.len())))
        .highlight_style(panel_highlight())
        .highlight_symbol(" ▶ ")
        .highlight_spacing(HighlightSpacing::Always);
    f.render_stateful_widget(list, area, &mut app.devices_state);
}

fn format_device_row(d: &DeviceInfo) -> Line<'static> {
    let mfr = if d.manufacturer.is_empty() {
        "(unknown)".to_string()
    } else {
        d.manufacturer.clone()
    };
    Line::from(vec![
        Span::styled(format!(" {:3} ", d.src), Style::default().fg(Color::Yellow)),
        Span::raw(format!(" {:20.20}", mfr)),
        Span::raw(format!(" {:24.24}", d.model)),
        Span::raw(format!(" sw {:14.14}", d.software)),
        Span::raw(format!(" pgns {:3}", d.pgn_count)),
        Span::raw(format!("  {}", d.installation)),
    ])
}

fn draw_nmea0183(f: &mut ratatui::Frame<'_>, area: Rect, app: &mut App, state: &AppState) {
    let rows = state.nmea0183_rows();
    // Friendly per-source labels from the device list.
    let labels: std::collections::HashMap<u8, String> = state
        .device_list()
        .into_iter()
        .map(|d| {
            let mfr = if d.manufacturer.is_empty() {
                "(unknown)".to_string()
            } else {
                d.manufacturer
            };
            let model = if d.model.is_empty() {
                String::new()
            } else {
                format!(" {}", d.model)
            };
            (d.src, format!("{mfr}{model}"))
        })
        .collect();
    let items: Vec<ListItem> = rows
        .iter()
        .map(|r| {
            let (tag, color) = if r.muted {
                ("muted", Color::Red)
            } else {
                (" on  ", Color::Green)
            };
            let line = match &r.sentence {
                None => Line::from(vec![
                    Span::styled(
                        format!(" src {:3} ", r.src),
                        Style::default()
                            .fg(Color::Yellow)
                            .add_modifier(Modifier::BOLD),
                    ),
                    Span::styled(
                        format!("[{tag}] "),
                        Style::default().fg(color).add_modifier(Modifier::BOLD),
                    ),
                    Span::styled(
                        labels.get(&r.src).cloned().unwrap_or_default(),
                        Style::default().add_modifier(Modifier::BOLD),
                    ),
                ]),
                Some(s) => Line::from(vec![
                    Span::raw("          "),
                    Span::raw(format!("{s:<5}")),
                    Span::styled(format!("[{tag}]"), Style::default().fg(color)),
                ]),
            };
            ListItem::new(line)
        })
        .collect();
    if !rows.is_empty() && app.nmea0183_state.selected().is_none() {
        app.nmea0183_state.select(Some(0));
    }
    let title = if rows.is_empty() {
        " NMEA 0183 filter (waiting for pipeline report…) ".to_string()
    } else {
        format!(" NMEA 0183 filter — {} sources ", state.nmea0183.len())
    };
    let list = List::new(items)
        .block(panel_block(title))
        .highlight_style(panel_highlight())
        .highlight_symbol(" ▶ ")
        .highlight_spacing(HighlightSpacing::Always);
    f.render_stateful_widget(list, area, &mut app.nmea0183_state);
}

/// Render the PGN Overrides view: one row per override the server is
/// currently applying, fed from its 262658 Report frames. Shows the
/// target src, PGN + description, and the requested interval (or OFF).
fn draw_overrides(f: &mut ratatui::Frame<'_>, area: Rect, app: &mut App, state: &AppState) {
    let rows = state.override_rows();
    let items: Vec<ListItem> = rows
        .iter()
        .map(|ov| {
            let interval = if ov.interval_ms == INTERVAL_OFF {
                "  OFF".to_string()
            } else {
                format!("{:>4} ms", ov.interval_ms)
            };
            let color = if ov.interval_ms == INTERVAL_OFF {
                Color::Red
            } else {
                Color::Green
            };
            let line = Line::from(vec![
                Span::styled(
                    format!(" src {:3} ", ov.src),
                    Style::default()
                        .fg(Color::Yellow)
                        .add_modifier(Modifier::BOLD),
                ),
                Span::styled(
                    format!("{:6} ", ov.pgn),
                    Style::default().add_modifier(Modifier::BOLD),
                ),
                Span::styled(format!("{:<9}", interval), Style::default().fg(color)),
                Span::raw(format!(" {}", ov.description)),
            ]);
            ListItem::new(line)
        })
        .collect();
    if !rows.is_empty() && app.overrides_state.selected().is_none() {
        app.overrides_state.select(Some(0));
    }
    let title = if rows.is_empty() {
        " PGN Overrides (none active) ".to_string()
    } else {
        format!(" PGN Overrides — {} active ", rows.len())
    };
    let list = List::new(items)
        .block(panel_block(title))
        .highlight_style(panel_highlight())
        .highlight_symbol(" ▶ ")
        .highlight_spacing(HighlightSpacing::Always);
    f.render_stateful_widget(list, area, &mut app.overrides_state);
}

fn draw_device_detail(
    f: &mut ratatui::Frame<'_>,
    area: Rect,
    app: &mut App,
    state: &AppState,
    src: u8,
) {
    let lists = state.pgn_lists_for_src(src);
    let bottom_h = if lists.is_empty() { 0 } else { 4 };
    let chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([Constraint::Min(1), Constraint::Length(bottom_h)])
        .split(area);

    let rows = app.detail_rows(src, state);
    let items: Vec<ListItem> = rows
        .iter()
        .map(|e| ListItem::new(format_entry_row(e, state)))
        .collect();
    let title = match state.device_list().iter().find(|d| d.src == src) {
        Some(d) => format!(
            " src {} — {} {} ({} entries) ",
            src,
            d.manufacturer,
            d.model,
            rows.len(),
        ),
        None => format!(" src {} ({} entries) ", src, rows.len()),
    };
    // Same trick as `draw_devices`: select row 0 on first sight of a
    // non-empty list so the marker is visible and the layout is
    // stable from the first frame.
    if !rows.is_empty() && app.detail_state.selected().is_none() {
        app.detail_state.select(Some(0));
    }
    let list = List::new(items)
        .block(panel_block(title))
        .highlight_style(panel_highlight())
        .highlight_symbol(" ▶ ")
        .highlight_spacing(HighlightSpacing::Always);
    f.render_stateful_widget(list, chunks[0], &mut app.detail_state);

    if bottom_h > 0 {
        draw_pgn_lists(f, chunks[1], &lists);
    }
}

/// Robust transmission interval for an entry: the median gap between its
/// most recent observations' wire timestamps. A first→last average
/// (`Entry::interval`) divides the whole capture span by the count, so
/// on an intermittent / merged log it dilutes every cadence with the
/// idle gaps between segments (a 60 s heartbeat reads as "every 361 s").
/// The median of a trailing window ignores those gap outliers and
/// reports the real cadence. Falls back to `Entry::interval` when the
/// records carry no parseable timestamps.
fn robust_interval(e: &Entry, state: &AppState) -> Option<std::time::Duration> {
    const WINDOW: usize = 64;
    // Last WINDOW+1 parseable wire timestamps (ms), oldest-first.
    let mut stamps: Vec<i64> = e
        .history_indices
        .iter()
        .rev()
        .take(WINDOW + 1)
        .filter_map(|&i| state.history.get(i))
        .filter_map(|h| h.timestamp.as_deref())
        .filter_map(canboat_core::parse_iso_ms)
        .collect();
    if stamps.len() < 2 {
        return e.interval();
    }
    stamps.reverse();
    // Positive consecutive gaps (a merged clock reset yields a negative
    // or huge gap — dropped / out-voted by the median).
    let mut deltas: Vec<i64> = stamps
        .windows(2)
        .map(|w| w[1] - w[0])
        .filter(|&d| d > 0)
        .collect();
    if deltas.is_empty() {
        return e.interval();
    }
    deltas.sort_unstable();
    let mid = deltas.len() / 2;
    let median = if deltas.len() % 2 == 1 {
        deltas[mid]
    } else {
        (deltas[mid - 1] + deltas[mid]) / 2
    };
    Some(std::time::Duration::from_millis(median as u64))
}

/// Whether canboat's schema marks this PGN as transmitted irregularly
/// (on request / on event) rather than on a fixed interval — e.g. ISO
/// Address Claim, Product / Configuration Information. For these a
/// measured "every X" is meaningless. Uses the first definition for the
/// PGN number (good enough; proprietary variants that share a number
/// mostly agree, and the standard on-request PGNs are unambiguous).
fn pgn_on_request(pgn: u32) -> bool {
    canboat_core::PgnDatabase::embedded(canboat_core::Units::Metric)
        .first_pgn(pgn)
        .and_then(|p| p.transmission_irregular)
        .unwrap_or(false)
}

fn format_entry_row(e: &Entry, state: &AppState) -> Line<'static> {
    let mode = state.status.mode;
    // `count == 0` is the sentinel for a synthetic row carrying no live
    // data (no measured interval, no meaningful age). Two kinds:
    //  * a *silenced* override — null `line` — drawn OFF in red;
    //  * an *advertised* PGN from a 126464 Transmit list — non-null
    //    synthetic `line` — drawn as a dim "advertised" row the user can
    //    still set an override on.
    if e.count == 0 {
        // Silenced (a live override set it OFF) vs advertised (named in a
        // 126464 list, never observed). Both are count==0; the live
        // override, not the synthetic line, tells them apart.
        let silenced = state
            .overrides
            .get(&(e.src, e.pgn))
            .is_some_and(|o| o.interval_ms == INTERVAL_OFF);
        let advertised = !silenced;
        let fallback = if advertised {
            "(advertised)"
        } else {
            "(disabled)"
        };
        let description = if e.description.is_empty() {
            fallback
        } else {
            e.description.as_str()
        };
        let dim = Style::default().fg(Color::DarkGray);
        let (state_label, state_style, note) = if advertised {
            ("—", dim, "(advertised)")
        } else {
            ("OFF", Style::default().fg(Color::Red), "(silenced)")
        };
        return Line::from(vec![
            Span::styled(format!(" {:6} ", e.pgn), Style::default().fg(Color::Yellow)),
            Span::styled(format!("{:14.14}", ""), dim),
            Span::styled(format!(" {description:30.30}"), dim),
            Span::styled(format!(" {state_label:>13}"), state_style),
            Span::styled(format!(" {:>10}", ""), dim),
            Span::styled(format!(" {note:>13}"), dim),
        ]);
    }
    let sec = e
        .secondary
        .as_deref()
        .map(|s| format!(":{s}"))
        .unwrap_or_default();
    let mut spans = vec![
        Span::styled(format!(" {:6} ", e.pgn), Style::default().fg(Color::Cyan)),
        Span::raw(format!("{:14.14}", sec)),
        Span::raw(format!(" {:30.30}", e.description)),
        // On-request / irregular PGNs (per canboat's schema) have no
        // cadence — any "every X" would be fiction — so label them
        // explicitly. Same 14-col width as " every <7>" so `count`
        // stays aligned.
        Span::raw(if pgn_on_request(e.pgn) {
            format!(" {:>13}", "on request")
        } else {
            format!(" every {}", format_interval(robust_interval(e, state)))
        }),
    ];
    // "age" is wall-clock seconds since `last_update` — a real
    // measurement in Live mode, but in Log mode it's just "how long
    // since we ingested this line", which is meaningless. Skip it.
    if mode == Mode::Live {
        let age = e.last_update.elapsed().as_secs();
        spans.push(Span::raw(format!(" age {age:>4}s")));
    }
    spans.push(Span::raw(format!(" count {:>6}", e.count)));
    Line::from(spans)
}

/// Build a placeholder `Entry` for a silenced override so it can
/// appear in the DeviceDetail row list alongside live entries. The
/// `count == 0` sentinel is what tells [`format_entry_row`] to draw
/// it as an OFF row and what tells the `o` key handler to read
/// mfr / industry / description back from the override file rather
/// than the (null) `line`.
fn synthesize_silenced_entry(ov: &OverrideRow) -> Entry {
    let now = std::time::Instant::now();
    // Carry the override's own proprietary header in a synthetic line so
    // `proprietary_codes` (advertised-row dedup + the override modal) reads
    // the manufacturer / industry back. A silenced vs advertised count==0
    // row is told apart by the live override, not by the line, so this is
    // free to be populated.
    let line = if is_proprietary_pgn(ov.pgn) {
        serde_json::json!({ "fields": {
            "Manufacturer Code": ov.manufacturer_code,
            "Industry Code": ov.industry_code,
        }})
    } else {
        serde_json::Value::Null
    };
    Entry {
        pgn: ov.pgn,
        src: ov.src,
        secondary: None,
        description: ov.description.clone(),
        line,
        last_update: now,
        count: 0,
        first_seen: now,
        first_stamp_ms: None,
        last_stamp_ms: None,
        // Silenced placeholders carry no observations — `count == 0`
        // is the sentinel that tells the row renderer to draw OFF
        // and the EntryDetail handler to skip drill-in.
        history_indices: Vec::new(),
    }
}

/// A proprietary PGN variant's manufacturer code — the `Match` on its first
/// field (`None` for a standard variant, which carries no such match).
fn variant_manufacturer(v: &canboat_core::PgnInfo) -> Option<u16> {
    v.fields
        .first()
        .and_then(|f| f.match_value)
        .and_then(|m| u16::try_from(m).ok())
}

/// A proprietary PGN variant's industry code — the `Match` on its third
/// field (the second is the reserved field between manufacturer and industry).
fn variant_industry(v: &canboat_core::PgnInfo) -> Option<u8> {
    v.fields
        .get(2)
        .and_then(|f| f.match_value)
        .and_then(|i| u8::try_from(i).ok())
}

/// The variant of a PGN a device *advertised* (in its 126464 Transmit list,
/// which names PGNs by number only) that applies to this device: for a
/// proprietary number, the variant whose manufacturer matches the device's
/// ISO-claim `device_mfr` (`None` when that manufacturer is unknown or the
/// device owns no variant of it, so a wrong manufacturer is never guessed);
/// for a standard number, its single definition.
fn advertised_variant(pgn: u32, device_mfr: Option<u16>) -> Option<&'static canboat_core::PgnInfo> {
    let db = canboat_core::PgnDatabase::embedded(canboat_core::Units::Metric);
    if !is_proprietary_pgn(pgn) {
        return db.first_pgn(pgn);
    }
    let mfr = device_mfr?;
    db.pgn_variants(pgn)
        .find(|v| variant_manufacturer(v) == Some(mfr))
}

/// The device's manufacturer code, for disambiguating a proprietary PGN a
/// 126464 list advertises by number only. Prefer its ISO-claim NAME; fall
/// back to the manufacturer decoded off any proprietary PGN it already
/// transmits (the NAME map needs the Unique Number, which some streams omit).
fn device_manufacturer(src: u8, state: &AppState) -> Option<u16> {
    if let Some(k) = state.src_to_name.get(&src) {
        return Some(k.manufacturer_code);
    }
    state
        .entries_for_src(src)
        .into_iter()
        .find_map(|e| proprietary_codes(e).0)
}

/// Build a placeholder `Entry` for a PGN a device advertised in its PGN
/// 126464 Transmit list but that we've never observed as live traffic.
/// Like [`synthesize_silenced_entry`] it uses the `count == 0` sentinel,
/// but carries a **non-null** synthetic `line` — which both distinguishes it
/// from a silenced row (null line) at render time and lets
/// [`proprietary_codes`] recover the manufacturer / industry so an override
/// can be built for a proprietary PGN we've only seen named, never decoded.
fn synthesize_advertised_entry(variant: &canboat_core::PgnInfo, src: u8) -> Entry {
    let now = std::time::Instant::now();
    let line = if is_proprietary_pgn(variant.pgn) {
        serde_json::json!({ "fields": {
            "Manufacturer Code": variant_manufacturer(variant),
            "Industry Code": variant_industry(variant),
        }})
    } else {
        serde_json::json!({ "fields": {} })
    };
    Entry {
        pgn: variant.pgn,
        src,
        secondary: None,
        description: variant.description.to_string(),
        line,
        last_update: now,
        count: 0,
        first_seen: now,
        first_stamp_ms: None,
        last_stamp_ms: None,
        history_indices: Vec::new(),
    }
}

/// Render a measured transmission interval as a fixed-width string
/// for the PGN row. The raw average bounces around as new samples
/// arrive (one slow frame at 100 ms nominal will read 102, 99, 103…),
/// which is distracting at a glance; snap each cadence to a step
/// sized for its magnitude so the displayed number stays put unless
/// the real rate actually changes.
///
/// Step ladder:
///
/// * `<  200 ms` → nearest 10 ms
/// * `<  1   s`  → nearest 50 ms
/// * `<  10  s`  → nearest 100 ms (displayed as ms)
/// * `≥ 10  s`   → nearest 1 s
///
/// `None` (count < 2) prints `—` so the column stays right-aligned.
fn format_interval(d: Option<std::time::Duration>) -> String {
    let Some(d) = d else {
        return format!("{:>7}", "—");
    };
    let ms = d.as_millis() as u64;
    let step = match ms {
        0..=199 => 10,
        200..=999 => 50,
        1_000..=9_999 => 100,
        _ => 1000,
    };
    let rounded = ((ms + step / 2) / step) * step;
    if rounded < 10_000 {
        format!("{rounded:>5}ms")
    } else {
        format!("{:>6.1}s", rounded as f32 / 1000.0)
    }
}

fn draw_pgn_lists(f: &mut ratatui::Frame<'_>, area: Rect, lists: &crate::tui::state::PgnLists) {
    let tx = lists
        .tx
        .iter()
        .map(|p| p.to_string())
        .collect::<Vec<_>>()
        .join(" ");
    let rx = lists
        .rx
        .iter()
        .map(|p| p.to_string())
        .collect::<Vec<_>>()
        .join(" ");
    let text = vec![
        Line::from(vec![
            Span::styled("TX: ", Style::default().fg(Color::Green)),
            Span::raw(tx),
        ]),
        Line::from(vec![
            Span::styled("RX: ", Style::default().fg(Color::Magenta)),
            Span::raw(rx),
        ]),
    ];
    let p = Paragraph::new(text)
        .block(panel_block(" PGN List (126464) "))
        .wrap(Wrap { trim: true });
    f.render_widget(p, area);
}

#[allow(clippy::too_many_arguments)]
fn draw_entry_detail(
    f: &mut ratatui::Frame<'_>,
    area: Rect,
    state: &AppState,
    src: u8,
    pgn: u32,
    secondary: Option<&str>,
    history_pos: usize,
    scroll: u16,
) {
    let key = (pgn, src, secondary.map(|s| s.to_string()));
    let Some(entry) = state.entries.get(&key) else {
        let p = Paragraph::new("(entry no longer in cache)")
            .style(Style::default().bg(PANEL_BG).fg(PANEL_FG))
            .block(panel_block(format!(" PGN {pgn} src {src} ")));
        f.render_widget(p, area);
        return;
    };
    // Render the observation indexed by `history_pos`; fall back to
    // `entry.line` (the cached latest) if the index has slipped out
    // of bounds — `draw` clamps but a synth row might still reach
    // here with an empty history_indices.
    let (line, timestamp) = entry
        .history_indices
        .get(history_pos)
        .and_then(|idx| state.history.get(*idx))
        .map(|h| (&h.line, h.timestamp.as_deref()))
        .unwrap_or((&entry.line, None));
    let pretty = serde_json::to_string_pretty(line).unwrap_or_else(|_| line.to_string());
    let total = entry.history_indices.len().max(1);
    let position = format!("[{}/{total}]", history_pos + 1);
    let stamp = timestamp.map(|t| format!(" {t}")).unwrap_or_default();
    let age_suffix = if state.status.mode == Mode::Live {
        format!("  age {}s", entry.last_update.elapsed().as_secs())
    } else {
        String::new()
    };
    let title = format!(
        " PGN {} src {} {}{}  {}  count {}{age_suffix} ",
        entry.pgn,
        entry.src,
        entry
            .secondary
            .as_deref()
            .map(|s| format!("[{s}] "))
            .unwrap_or_default(),
        position,
        stamp,
        entry.count,
    );
    let p = Paragraph::new(pretty)
        .style(Style::default().bg(PANEL_BG).fg(PANEL_FG))
        .block(panel_block(title))
        .wrap(Wrap { trim: false })
        .scroll((scroll, 0));
    f.render_widget(p, area);
}

/// Chronological table view — one row per observation in
/// `AppState::history`. Columns: index / timestamp / src / pgn /
/// description. Enter drills into the corresponding EntryDetail
/// screen, positioned to that specific observation.
fn draw_time_view(f: &mut ratatui::Frame<'_>, area: Rect, app: &mut App, state: &AppState) {
    // Count (cheap) rather than materialise the whole visible set — a
    // 700 MB capture has millions of rows and this runs every frame.
    let total = app.visible_history_len(state);
    // Clamp the cursor to the (possibly reduced) filter subset.
    if total == 0 {
        app.time_state.select(None);
        app.time_offset = 0;
    } else {
        let sel = app.time_state.selected().unwrap_or(0).min(total - 1);
        app.time_state.select(Some(sel));
    }

    // Keep the selected row inside the viewport, then clamp the offset so
    // we never scroll past the tail.
    let view_h = area.height.saturating_sub(2) as usize; // minus the border
    if view_h > 0 && total > 0 {
        let sel = app.time_state.selected().unwrap_or(0);
        if sel < app.time_offset {
            app.time_offset = sel;
        } else if sel >= app.time_offset + view_h {
            app.time_offset = sel + 1 - view_h;
        }
        app.time_offset = app.time_offset.min(total.saturating_sub(view_h));
    }

    // Build ListItems only for the on-screen window.
    let window = app.visible_window(state, app.time_offset, view_h.max(1));
    let items: Vec<ListItem> = window
        .iter()
        .filter_map(|hi| state.history.get(*hi).map(|h| (hi, h)))
        .map(|(hi, h)| ListItem::new(format_time_row(*hi, h)))
        .collect();
    let mut title_parts = vec![format!("Timeline ({total}")];
    if state.history.len() != total {
        title_parts.push(format!(" of {}", state.history.len()));
    }
    title_parts.push(")".to_string());
    if let Some(list) = &app.filter_pgns {
        title_parts.push(format!(
            "  pgns: {}",
            list.iter()
                .map(|p| p.to_string())
                .collect::<Vec<_>>()
                .join(",")
        ));
    }
    if let Some(list) = &app.filter_srcs {
        title_parts.push(format!(
            "  srcs: {}",
            list.iter()
                .map(|s| s.to_string())
                .collect::<Vec<_>>()
                .join(",")
        ));
    }
    if let Some(q) = &app.search_query {
        title_parts.push(format!("  /{q}"));
    }
    let title: String = format!(" {} ", title_parts.concat());
    let list_widget = List::new(items)
        .block(panel_block(title))
        .highlight_style(panel_highlight())
        .highlight_symbol(" ▶ ")
        .highlight_spacing(HighlightSpacing::Always);
    // The window items start at `time_offset`, so highlight the selected
    // row relative to the window's top.
    let mut window_state = ListState::default();
    if let Some(sel) = app.time_state.selected() {
        window_state.select(Some(sel.saturating_sub(app.time_offset)));
    }
    f.render_stateful_widget(list_widget, area, &mut window_state);
}

/// `top`-style PGN-load view — one row per PGN, busiest first, with a
/// rate bar-graph column. Enter drills into the timeline filtered to
/// that PGN.
fn draw_pgn_top(f: &mut ratatui::Frame<'_>, area: Rect, app: &mut App, state: &AppState) {
    let rows = state.pgn_load_rows();
    if !rows.is_empty() && app.pgn_top_state.selected().is_none() {
        app.pgn_top_state.select(Some(0));
    }
    if let Some(sel) = app.pgn_top_state.selected()
        && sel >= rows.len()
    {
        app.pgn_top_state
            .select(rows.len().checked_sub(1).or(Some(0)));
    }
    // Scale each bar against the busiest PGN so the top row is full.
    let max_rate = rows.iter().map(|r| r.rate).fold(0.0_f32, f32::max);
    let total_rate: f32 = rows.iter().map(|r| r.rate).sum();
    let items: Vec<ListItem> = rows
        .iter()
        .map(|r| ListItem::new(format_pgn_top_row(r, max_rate)))
        .collect();
    let title = format!(
        " PGN Load — {} pgns, {:.0} msg/s total ",
        rows.len(),
        total_rate
    );
    // A solid-background highlight (not REVERSED): REVERSED would flip
    // the green rate bar to the panel background, hiding the busiest
    // row's full-width bar. The green selection bar recolours the bar's
    // blocks to white instead, so the bar length stays visible.
    let list = List::new(items)
        .block(panel_block(title))
        .highlight_style(
            Style::default()
                .bg(menu::SELECT_BG)
                .fg(menu::SELECT_FG)
                .add_modifier(Modifier::BOLD),
        )
        .highlight_symbol(" ▶ ")
        .highlight_spacing(HighlightSpacing::Always);
    f.render_stateful_widget(list, area, &mut app.pgn_top_state);
}

/// One row on the PGN-load view: pgn / description / #sources / count /
/// rate / proportional bar.
fn format_pgn_top_row(r: &PgnLoadRow, max_rate: f32) -> Line<'static> {
    const BAR_W: usize = 24;
    let filled = if max_rate > 0.0 {
        ((r.rate / max_rate) * BAR_W as f32).round() as usize
    } else {
        0
    }
    .min(BAR_W);
    let bar = format!("{}{}", "█".repeat(filled), "░".repeat(BAR_W - filled));
    let rate = if r.rate >= 1.0 {
        format!("{:>6.1}/s", r.rate)
    } else if r.rate > 0.0 {
        format!("{:>6.2}/s", r.rate)
    } else {
        format!("{:>9}", "—")
    };
    Line::from(vec![
        Span::styled(format!(" {:6} ", r.pgn), Style::default().fg(Color::Cyan)),
        Span::raw(format!("{:32.32}", r.description)),
        Span::styled(
            format!(" {:>3} src", r.sources),
            Style::default().fg(Color::Yellow),
        ),
        Span::raw(format!(" {:>9}", r.count)),
        Span::raw(format!("  {rate}  ")),
        Span::styled(bar, Style::default().fg(Color::Green)),
    ])
}

/// Canonical row string for search matching — a plain-text form of
/// [`format_time_row`] without ANSI/ratatui styling. Kept in sync so
/// searching for whatever the user sees on-screen finds the row.
fn row_search_string(h: &crate::tui::state::HistoryRecord) -> String {
    let stamp = h.timestamp.as_deref().unwrap_or("");
    let sec = h.secondary.as_deref().unwrap_or("");
    format!(
        "{stamp} src {} pgn {} {sec} {}",
        h.src, h.pgn, h.description
    )
}

/// One row on the `TimeView` screen — index, timestamp, src, pgn,
/// and description columns. Timestamp column is right-padded to a
/// fixed width so `src`/`pgn` line up column-wise across rows.
fn format_time_row(idx: usize, h: &crate::tui::state::HistoryRecord) -> Line<'static> {
    // Prefer the wire timestamp from the analyzer JSON; fall back
    // to relative wall-clock age (`+<N>s`) computed from `seen_at`
    // when the record didn't carry one.
    let stamp: String = match h.timestamp.as_deref() {
        Some(s) if !s.is_empty() => s.to_string(),
        _ => format!("+{}s", h.seen_at.elapsed().as_secs()),
    };
    let sec = h
        .secondary
        .as_deref()
        .map(|s| format!(":{s}"))
        .unwrap_or_default();
    Line::from(vec![
        Span::styled(format!(" {idx:>6}  "), Style::default().fg(Color::DarkGray)),
        Span::styled(format!("{stamp:<24}"), Style::default().fg(Color::White)),
        Span::styled(
            format!(" src {:3}", h.src),
            Style::default().fg(Color::Yellow),
        ),
        Span::styled(
            format!("  {:6}{sec:<10}", h.pgn),
            Style::default().fg(Color::Cyan),
        ),
        Span::raw(format!("  {}", h.description)),
    ])
}

/// A content-window block: blue panel, cyan double border, bold white
/// title. Every full-screen list / paragraph uses this so the windows
/// read as one Turbo-Pascal desktop.
fn panel_block(title: impl Into<String>) -> Block<'static> {
    Block::default()
        .borders(Borders::ALL)
        .border_type(BorderType::Double)
        .border_style(Style::default().fg(PANEL_BORDER).bg(PANEL_BG))
        .style(Style::default().bg(PANEL_BG).fg(PANEL_FG))
        .title(Span::styled(
            title.into(),
            Style::default()
                .fg(PANEL_TITLE)
                .bg(PANEL_BG)
                .add_modifier(Modifier::BOLD),
        ))
}

/// Selection-bar style for list rows inside a content window. Uses
/// `REVERSED` rather than a fixed background so it stays legible over
/// rows whose spans carry their own colours (several columns are cyan,
/// which would vanish on a solid cyan bar).
fn panel_highlight() -> Style {
    Style::default().add_modifier(Modifier::REVERSED)
}

/// A dialog block: grey surface, cyan double border, bold cyan title —
/// the raised-dialog look sitting on the blue desktop.
fn dialog_block(title: impl Into<String>) -> Block<'static> {
    Block::default()
        .borders(Borders::ALL)
        .border_type(BorderType::Double)
        .border_style(Style::default().fg(Color::Cyan).bg(SURFACE_BG))
        .style(Style::default().bg(SURFACE_BG).fg(SURFACE_FG))
        .title(Span::styled(
            title.into(),
            Style::default()
                .fg(SURFACE_ACCENT)
                .bg(SURFACE_BG)
                .add_modifier(Modifier::BOLD),
        ))
}

/// Compute a centered rect of at most `max_w` × `max_h`, clamped to
/// `area`. Used by every modal so they end up in the same place.
fn centered_rect(area: Rect, max_w: u16, max_h: u16) -> Rect {
    let w = max_w.min(area.width.saturating_sub(2));
    let h = max_h.min(area.height.saturating_sub(2));
    let x = area.x + (area.width.saturating_sub(w)) / 2;
    let y = area.y + (area.height.saturating_sub(h)) / 2;
    Rect {
        x,
        y,
        width: w,
        height: h,
    }
}

fn draw_modal(f: &mut ratatui::Frame<'_>, area: Rect, modal: &OverrideModal) {
    let rect = centered_rect(area, 60, 8);
    menu::draw_shadow(f, rect, area);
    f.render_widget(Clear, rect);
    let mfr = modal
        .manufacturer_code
        .map(|m| format!(" mfr={m}"))
        .unwrap_or_default();
    let ind = modal
        .industry_code
        .map(|i| format!(" ind={i}"))
        .unwrap_or_default();
    let title = match &modal.description {
        Some(d) if !d.is_empty() => {
            format!("Override PGN {} ({d}) on src {}", modal.pgn, modal.src)
        }
        _ => format!("Override PGN {} on src {}", modal.pgn, modal.src),
    };
    let text = vec![
        Line::from(format!("{title}{mfr}{ind}")),
        Line::from(""),
        Line::from(format!("New interval (ms): {}_", modal.input)),
        Line::from(""),
        Line::from("Enter to send • Esc to cancel"),
        Line::from(format!("{INTERVAL_OFF} = stop transmitting")),
    ];
    let p = Paragraph::new(text)
        .style(Style::default().bg(SURFACE_BG).fg(SURFACE_FG))
        .block(dialog_block(" PGN Override "))
        .wrap(Wrap { trim: true });
    f.render_widget(p, rect);
}

/// Centered "Connecting…" overlay shown at startup until both the
/// snapshot and live-stream connections terminate (success or
/// failure). The body live-updates from `state.status` so the user
/// sees each connection's progress.
fn draw_connecting_modal(f: &mut ratatui::Frame<'_>, area: Rect, state: &AppState) {
    let rect = centered_rect(area, 64, 10);
    menu::draw_shadow(f, rect, area);
    f.render_widget(Clear, rect);
    let s = &state.status;
    let snap_status = if s.snapshot_loaded {
        Span::styled(
            "✓ loaded",
            Style::default()
                .fg(Color::Green)
                .add_modifier(Modifier::BOLD),
        )
    } else if s
        .last_error
        .as_deref()
        .is_some_and(|e| e.starts_with("snapshot"))
    {
        Span::styled(
            "✗ failed",
            Style::default().fg(Color::Red).add_modifier(Modifier::BOLD),
        )
    } else {
        Span::styled("… connecting", Style::default().fg(Color::Yellow))
    };
    let stream_status = if s.stream_connected {
        Span::styled(
            "✓ connected",
            Style::default()
                .fg(Color::Green)
                .add_modifier(Modifier::BOLD),
        )
    } else if s
        .last_error
        .as_deref()
        .is_some_and(|e| e.starts_with("stream"))
    {
        Span::styled(
            "✗ failed",
            Style::default().fg(Color::Red).add_modifier(Modifier::BOLD),
        )
    } else {
        Span::styled("… connecting", Style::default().fg(Color::Yellow))
    };
    let text = vec![
        Line::from(format!("Connecting to {}", s.host)),
        Line::from(""),
        Line::from(vec![
            Span::raw(format!("  Snapshot  :{:<5}  ", s.snapshot_port)),
            snap_status,
        ]),
        Line::from(vec![
            Span::raw(format!("  Stream    :{:<5}  ", s.stream_port)),
            stream_status,
        ]),
        Line::from(""),
        Line::from(Span::styled(
            "Press any key to continue (auto-dismisses when both up)",
            Style::default().bg(SURFACE_BG).fg(menu::DISABLED_FG),
        )),
    ];
    let p = Paragraph::new(text)
        .style(Style::default().bg(SURFACE_BG).fg(SURFACE_FG))
        .block(dialog_block(" canboat-tui "))
        .wrap(Wrap { trim: true });
    f.render_widget(p, rect);
}

/// Centered red-bordered modal shown whenever
/// `state.status.last_error` carries a value the user hasn't yet
/// acknowledged. Dismissed with any keystroke.
fn draw_error_modal(f: &mut ratatui::Frame<'_>, area: Rect, message: &str) {
    // Word-wrapped lines fit in a generous box; cap height at 12 so a
    // multi-paragraph error never eats the whole terminal.
    let rect = centered_rect(area, 72, 12);
    menu::draw_shadow(f, rect, area);
    f.render_widget(Clear, rect);
    let surface = Style::default().bg(SURFACE_BG).fg(SURFACE_FG);
    let text = vec![
        Line::from(Span::styled(
            "Fatal error",
            surface.fg(Color::Red).add_modifier(Modifier::BOLD),
        )),
        Line::from(""),
        Line::from(Span::styled(message.to_string(), surface)),
        Line::from(""),
        Line::from(Span::styled(
            "Press any key to dismiss",
            surface.fg(SURFACE_ACCENT),
        )),
    ];
    let p = Paragraph::new(text)
        .style(surface)
        .block(
            Block::default()
                .borders(Borders::ALL)
                .border_type(BorderType::Double)
                .border_style(surface.fg(Color::Red))
                .style(surface)
                .title(Span::styled(
                    " Error ",
                    surface.fg(Color::Red).add_modifier(Modifier::BOLD),
                )),
        )
        .wrap(Wrap { trim: true });
    f.render_widget(p, rect);
}

/// Total intro-animation frames (~2.6 s at the 60 ms splash tick).
const SPLASH_FRAMES: u16 = 43;
/// Frames spent zooming the window open; then the banner holds.
const SPLASH_GROW: u16 = 15;
/// Frame at which the banner hold gives way to the animated
/// `LOADING…` / `CONNECTING…` tail (~0.8 s of the total).
const SPLASH_LOAD_START: u16 = 30;

/// 5-row block-letter rows for one banner glyph (only the letters in
/// `CANBOAT` are defined; anything else renders blank).
fn banner_glyph(c: char) -> [&'static str; 5] {
    match c {
        'C' => ["█████", "█    ", "█    ", "█    ", "█████"],
        'A' => [" ███ ", "█   █", "█████", "█   █", "█   █"],
        'N' => ["█   █", "██  █", "█ █ █", "█  ██", "█   █"],
        'B' => ["████ ", "█   █", "████ ", "█   █", "████ "],
        'O' => [" ███ ", "█   █", "█   █", "█   █", " ███ "],
        'T' => ["█████", "  █  ", "  █  ", "  █  ", "  █  "],
        _ => ["     ", "     ", "     ", "     ", "     "],
    }
}

/// Assemble the 5 banner rows for `word`, one blank column between
/// glyphs.
fn banner_lines(word: &str) -> Vec<String> {
    let mut rows = vec![String::new(); 5];
    for (i, ch) in word.chars().enumerate() {
        let g = banner_glyph(ch);
        for (r, row) in rows.iter_mut().enumerate() {
            if i > 0 {
                row.push(' ');
            }
            row.push_str(g[r]);
        }
    }
    rows
}

/// Startup intro: a black screen with a blue window that zooms open
/// behind a big block-letter `CANBOAT` banner and a C64-style boot
/// block, wrapped in a flashing Spectrum/C64 loading border. The tail
/// shows `verb` ("LOADING" / "CONNECTING") with animated dots. Advances
/// one step per repaint; any key skips it.
fn draw_splash(f: &mut ratatui::Frame<'_>, area: Rect, frame: u16, verb: &str) {
    // Spectrum/C64 loading-border colours — the frame flashes through
    // them like a tape load.
    const BORDER: [Color; 7] = [
        Color::Rgb(0xff, 0x00, 0x00),
        Color::Rgb(0xff, 0xff, 0x00),
        Color::Rgb(0x00, 0xff, 0xff),
        Color::Rgb(0x00, 0xff, 0x00),
        Color::Rgb(0xff, 0x00, 0xff),
        Color::Rgb(0xff, 0xff, 0xff),
        Color::Rgb(0x00, 0x00, 0xff),
    ];
    let light_blue = Color::Rgb(0x8b, 0x8b, 0xff);
    let blue = Style::default().bg(DESKTOP_BG);

    // Black backdrop.
    f.render_widget(
        Block::default().style(Style::default().bg(Color::Black)),
        area,
    );

    // Zoom the window open from the centre.
    let p = f32::from(frame.min(SPLASH_GROW)) / f32::from(SPLASH_GROW);
    let bw = ((area.width as f32 * p).ceil() as u16).clamp(2, area.width);
    let bh = ((area.height as f32 * p).ceil() as u16).clamp(2, area.height);
    let bx = area.x + (area.width - bw) / 2;
    let by = area.y + (area.height - bh) / 2;
    let outer = Rect {
        x: bx,
        y: by,
        width: bw,
        height: bh,
    };
    // Flashing loading border, then the blue interior on top.
    f.render_widget(Clear, outer);
    f.render_widget(
        Block::default().style(Style::default().bg(BORDER[frame as usize % BORDER.len()])),
        outer,
    );
    if bw <= 2 || bh <= 2 {
        return;
    }
    let inner = Rect {
        x: bx + 1,
        y: by + 1,
        width: bw - 2,
        height: bh - 2,
    };
    f.render_widget(Block::default().style(blue), inner);

    // Hold the content back until the window can actually hold it.
    let banner = banner_lines("CANBOAT");
    let banner_w = banner.first().map(|s| s.chars().count()).unwrap_or(0) as u16;
    if inner.width < banner_w || inner.height < 12 {
        return;
    }

    let mut lines: Vec<Line> = banner
        .iter()
        .map(|row| {
            Line::from(Span::styled(
                row.clone(),
                blue.fg(light_blue).add_modifier(Modifier::BOLD),
            ))
        })
        .collect();
    lines.push(Line::from(""));
    // C64 boot-screen homage.
    lines.push(Line::from(Span::styled(
        format!("**** CANBOAT-TUI V{} ****", env!("CARGO_PKG_VERSION")),
        blue.fg(light_blue),
    )));
    lines.push(Line::from(Span::styled(
        "(C) 2009-2026 KEES VERRUIJT, HARLINGEN, THE NETHERLANDS",
        blue.fg(light_blue),
    )));
    lines.push(Line::from(""));
    // READY. with a blinking block cursor underneath.
    lines.push(Line::from(Span::styled("READY.", blue.fg(light_blue))));
    let cursor = if (frame / 3).is_multiple_of(2) {
        "█"
    } else {
        " "
    };
    lines.push(Line::from(Span::styled(cursor, blue.fg(light_blue))));
    lines.push(Line::from(""));
    // Bottom line: blink "PRESS ANY KEY" during the banner hold, then
    // switch to an animated "LOADING …" / "CONNECTING …" tail.
    let bottom = if frame >= SPLASH_LOAD_START {
        let dots = ".".repeat(1 + (frame as usize / 2) % 3);
        format!("{verb} {dots}")
    } else if (frame / 4).is_multiple_of(2) {
        "PRESS ANY KEY".to_string()
    } else {
        String::new()
    };
    lines.push(Line::from(Span::styled(
        bottom,
        blue.fg(Color::Rgb(0xff, 0xff, 0xff))
            .add_modifier(Modifier::BOLD),
    )));

    let text_area = Rect {
        x: inner.x,
        y: inner.y + 1,
        width: inner.width,
        height: inner.height.saturating_sub(1),
    };
    let para = Paragraph::new(lines)
        .alignment(Alignment::Center)
        .style(blue);
    f.render_widget(para, text_area);
}

/// Swap a trailing `.json` / `.raw` extension on `input` for `ext`
/// (appending if there's no recognised capture extension to replace),
/// leaving unrelated dotted names alone.
fn swap_extension(input: &str, ext: &str) -> String {
    match input.rsplit_once('.') {
        Some((stem, old))
            if old.eq_ignore_ascii_case("json") || old.eq_ignore_ascii_case("raw") =>
        {
            format!("{stem}.{ext}")
        }
        _ => format!("{input}.{ext}"),
    }
}

/// Progress bar for a running capture save. A grey dialog with a filled
/// bar and a `done / total (pct%)` readout.
fn draw_progress_modal(f: &mut ratatui::Frame<'_>, area: Rect, p: &Progress) {
    let rect = centered_rect(area, 56, 7);
    menu::draw_shadow(f, rect, area);
    f.render_widget(Clear, rect);
    let surface = Style::default().bg(SURFACE_BG).fg(SURFACE_FG);

    let block = dialog_block(" Saving ");
    let inner = block.inner(rect);
    f.render_widget(block, rect);

    // `checked_div` yields None (→ fall back to "full") when total is 0.
    let pct = p
        .done
        .saturating_mul(100)
        .checked_div(p.total)
        .unwrap_or(100) as u16;
    let bar_w = inner.width.saturating_sub(2) as usize;
    let filled = bar_w
        .saturating_mul(p.done)
        .checked_div(p.total)
        .unwrap_or(bar_w);
    let bar: String = std::iter::repeat_n('█', filled)
        .chain(std::iter::repeat_n('░', bar_w.saturating_sub(filled)))
        .collect();

    let text = vec![
        Line::from(Span::styled(format!(" {}", p.label), surface)),
        Line::from(""),
        Line::from(Span::styled(
            format!(" {bar}"),
            surface.fg(SURFACE_ACCENT).add_modifier(Modifier::BOLD),
        )),
        Line::from(Span::styled(
            format!(" {} / {} records  ({pct}%)", p.done, p.total),
            surface,
        )),
    ];
    f.render_widget(Paragraph::new(text).style(surface), inner);
}

/// File ▸ Connect dialog: a grey dialog with a single editable endpoint
/// line.
fn draw_io_modal(f: &mut ratatui::Frame<'_>, area: Rect, m: &IoModal) {
    let rect = centered_rect(area, 68, 8);
    menu::draw_shadow(f, rect, area);
    f.render_widget(Clear, rect);
    let surface = Style::default().bg(SURFACE_BG).fg(SURFACE_FG);
    let text = vec![
        Line::from(Span::styled(
            "Endpoint  (host [snapshot-port [stream-port]]):",
            surface,
        )),
        Line::from(""),
        Line::from(Span::styled(
            format!("{}_", m.input),
            surface.fg(SURFACE_ACCENT).add_modifier(Modifier::BOLD),
        )),
        Line::from(""),
        Line::from(Span::styled("Enter to confirm • Esc to cancel", surface)),
        Line::from(Span::styled(
            "Ports default to 2597 / 2598 when omitted.",
            surface.fg(menu::DISABLED_FG),
        )),
    ];
    let p = Paragraph::new(text)
        .style(surface)
        .block(dialog_block(" Connect "))
        .wrap(Wrap { trim: true });
    f.render_widget(p, rect);
}

/// Left-align `s` in a `w`-column field: pad with spaces, or truncate
/// with a trailing `…` when it's too long (char-aware).
fn fit_width(s: &str, w: usize) -> String {
    let n = s.chars().count();
    if n > w {
        let keep = w.saturating_sub(1);
        format!("{}…", s.chars().take(keep).collect::<String>())
    } else {
        format!("{s:<w$}")
    }
}

/// Compact human-readable file size (`512B`, `1.4K`, `700M`, `12.3G`),
/// binary (1024) units.
fn human_size(bytes: u64) -> String {
    const UNITS: [&str; 5] = ["B", "K", "M", "G", "T"];
    let mut v = bytes as f64;
    let mut u = 0;
    while v >= 1024.0 && u < UNITS.len() - 1 {
        v /= 1024.0;
        u += 1;
    }
    if u == 0 {
        format!("{bytes}B")
    } else if v >= 100.0 {
        format!("{v:.0}{}", UNITS[u])
    } else {
        format!("{v:.1}{}", UNITS[u])
    }
}

/// Format a file's modified time as `YYYY-MM-DD HH:MM` in the machine's
/// local timezone (chrono resolves the OS zone + DST for the instant).
fn format_mtime(t: std::time::SystemTime) -> String {
    chrono::DateTime::<chrono::Local>::from(t)
        .format("%Y-%m-%d %H:%M")
        .to_string()
}

/// Keep the tail of `s` (the informative end of a path), prefixing `…`
/// when it doesn't fit in `max` columns.
fn truncate_start(s: &str, max: usize) -> String {
    let count = s.chars().count();
    if count <= max {
        return s.to_string();
    }
    let skip = count - max.saturating_sub(1);
    format!("…{}", s.chars().skip(skip).collect::<String>())
}

/// File ▸ Load / Save browser: a grey dialog listing the current
/// directory (parent, sub-dirs, then files), path in the title. In Save
/// mode it also shows an editable filename + a format selector below the
/// listing.
fn draw_file_browser(f: &mut ratatui::Frame<'_>, area: Rect, fb: &FileBrowser) {
    let surface = Style::default().bg(SURFACE_BG).fg(SURFACE_FG);
    // Reserve rows below the listing: 1 hint (+ name + format in Save).
    let reserved: u16 = if fb.save { 3 } else { 1 };
    let w = 76.min(area.width.saturating_sub(4)).max(24);
    let h = (fb.entries.len() as u16 + 2 + reserved).clamp(8, area.height.saturating_sub(4));
    let rect = centered_rect(area, w, h);
    menu::draw_shadow(f, rect, area);
    f.render_widget(Clear, rect);

    let verb = if fb.save { "Save" } else { "Load" };
    let cwd = fb.cwd.display().to_string();
    let title = if fb.error.is_some() {
        format!(" {verb} — {} [unreadable] ", truncate_start(&cwd, 30))
    } else {
        format!(
            " {verb} — {} ",
            truncate_start(&cwd, rect.width.saturating_sub(12) as usize)
        )
    };

    // Draw the framed background first, then place content in its inner
    // area so the reserved rows sit inside the border.
    let block = dialog_block(title);
    let inner = block.inner(rect);
    f.render_widget(block, rect);

    let list_h = inner.height.saturating_sub(reserved);
    let list_area = Rect {
        x: inner.x,
        y: inner.y,
        width: inner.width,
        height: list_h,
    };
    // Columns: name (left) | size (right) | modified date. Drop the two
    // trailing columns on a very narrow dialog so the name still shows.
    const SIZE_W: usize = 8;
    const DATE_W: usize = 16; // "YYYY-MM-DD HH:MM"
    let avail = list_area.width.saturating_sub(3) as usize; // minus " ▶ "
    let show_meta = avail >= SIZE_W + DATE_W + 6;
    let name_w = if show_meta {
        avail - SIZE_W - DATE_W - 2
    } else {
        avail
    };
    let items: Vec<ListItem> = fb
        .entries
        .iter()
        .map(|e| {
            let display = if e.is_dir {
                format!("{}/", e.name)
            } else {
                e.name.clone()
            };
            let name_style = if e.is_dir {
                surface.fg(SURFACE_ACCENT).add_modifier(Modifier::BOLD)
            } else {
                surface
            };
            let mut spans = vec![Span::styled(fit_width(&display, name_w), name_style)];
            if show_meta {
                let size_str = if e.name == ".." {
                    String::new()
                } else if e.is_dir {
                    "<DIR>".to_string()
                } else {
                    human_size(e.size)
                };
                let date_str = e.modified.map(format_mtime).unwrap_or_default();
                spans.push(Span::styled(
                    format!(" {size_str:>SIZE_W$} {date_str:<DATE_W$}"),
                    surface.fg(menu::DISABLED_FG),
                ));
            }
            ListItem::new(Line::from(spans))
        })
        .collect();
    let mut list_state = ListState::default();
    if !fb.entries.is_empty() {
        list_state.select(Some(fb.selected));
    }
    let list = List::new(items)
        .style(surface)
        .highlight_style(Style::default().add_modifier(Modifier::REVERSED))
        .highlight_symbol(" ▶ ")
        .highlight_spacing(HighlightSpacing::Always);
    f.render_stateful_widget(list, list_area, &mut list_state);

    let mut y = inner.y + list_h;
    let mut row = |f: &mut ratatui::Frame<'_>, line: Line<'static>| {
        f.render_widget(
            Paragraph::new(line).style(surface),
            Rect {
                x: inner.x,
                y,
                width: inner.width,
                height: 1,
            },
        );
        y += 1;
    };
    if fb.save {
        row(
            f,
            Line::from(vec![
                Span::styled(" Name: ", surface),
                Span::styled(
                    format!("{}_", fb.filename),
                    surface.fg(SURFACE_ACCENT).add_modifier(Modifier::BOLD),
                ),
            ]),
        );
        let sel = surface
            .fg(SURFACE_ACCENT)
            .add_modifier(Modifier::BOLD | Modifier::REVERSED);
        let (analysed, raw) = match fb.format {
            SaveFormat::Analysed => (sel, surface),
            SaveFormat::Raw => (surface, sel),
        };
        row(
            f,
            Line::from(vec![
                Span::styled(" Format (Tab): ", surface),
                Span::styled(" Analysed (.json) ", analysed),
                Span::styled("  ", surface),
                Span::styled(" Raw (.raw) ", raw),
            ]),
        );
        row(
            f,
            Line::from(Span::styled(
                " ↑↓ move | → open dir | ← up | type name | Enter save | Esc cancel",
                surface.fg(SURFACE_ACCENT),
            )),
        );
    } else {
        row(
            f,
            Line::from(Span::styled(
                " ↑↓ move | Enter open | ← parent | Esc cancel",
                surface.fg(SURFACE_ACCENT),
            )),
        );
    }
}

/// Help / About overlay (F1 or Help ▸ Keys). A grey dialog listing the
/// global keys and per-view shortcuts. Dismissed with any key.
fn draw_help_modal(f: &mut ratatui::Frame<'_>, area: Rect) {
    let rect = centered_rect(area, 66, 22);
    menu::draw_shadow(f, rect, area);
    f.render_widget(Clear, rect);
    let surface = Style::default().bg(SURFACE_BG).fg(SURFACE_FG);
    let head = surface.fg(SURFACE_ACCENT).add_modifier(Modifier::BOLD);
    let text = vec![
        Line::from(Span::styled(
            format!("canboat-tui v{}", env!("CARGO_PKG_VERSION")),
            head,
        )),
        Line::from(Span::styled("Interactive NMEA 2000 bus viewer", surface)),
        Line::from(""),
        Line::from(Span::styled("Menu", head)),
        Line::from(Span::styled("  F10            open the menu bar", surface)),
        Line::from(Span::styled(
            "  Alt-F/V/D/S/H  jump straight to a menu",
            surface,
        )),
        Line::from(Span::styled(
            "  ←→ ↑↓ Enter    navigate • Esc closes",
            surface,
        )),
        Line::from(""),
        Line::from(Span::styled("Views", head)),
        Line::from(Span::styled(
            "  d devices   t timeline   p PGN load   n NMEA 0183",
            surface,
        )),
        Line::from(""),
        Line::from(Span::styled("Navigation", head)),
        Line::from(Span::styled(
            "  ↑↓ / j k  move    Enter  open / drill in    Esc  back",
            surface,
        )),
        Line::from(""),
        Line::from(Span::styled("Device (live bus)", head)),
        Line::from(Span::styled(
            "  i  ISO Request 126464     o  override interval",
            surface,
        )),
        Line::from(""),
        Line::from(Span::styled("Timeline", head)),
        Line::from(Span::styled(
            "  / search   n/N next/prev   f filter pgns   s filter srcs",
            surface,
        )),
        Line::from(""),
        Line::from(Span::styled(
            "Press any key to close",
            surface.fg(SURFACE_ACCENT),
        )),
    ];
    let p = Paragraph::new(text)
        .style(surface)
        .block(dialog_block(" Help "))
        .wrap(Wrap { trim: false });
    f.render_widget(p, rect);
}

/// One-line text-prompt bar — replaces the hint bar while `/` or
/// `f` is active. Renders `<prefix> <buffer>_` with a fake cursor
/// glyph at the end of the buffer.
fn draw_text_prompt(f: &mut ratatui::Frame<'_>, area: Rect, prompt: &TextPrompt) {
    let prefix = match prompt.kind {
        TextPromptKind::Search => "/",
        TextPromptKind::FilterPgns => "filter pgns: ",
    };
    let text = format!(" {prefix}{}_", prompt.buffer);
    let p = Paragraph::new(text).style(
        Style::default()
            .fg(Color::Black)
            .bg(Color::Yellow)
            .add_modifier(Modifier::BOLD),
    );
    f.render_widget(p, area);
}

/// Source-select checkbox modal — shown while the `s` prompt is
/// active on TimeView. Each row is `[x] <label>`; `Space` toggles
/// the cursor's row, `a` toggles all, Enter applies, Esc cancels.
fn draw_src_select_modal(f: &mut ratatui::Frame<'_>, area: Rect, sel: &SrcSelect) {
    let w = 60.min(area.width.saturating_sub(2));
    let h = (sel.sources.len() as u16 + 6)
        .min(area.height.saturating_sub(2))
        .max(6);
    let rect = centered_rect(area, w, h);
    let surface = Style::default().bg(SURFACE_BG).fg(SURFACE_FG);
    menu::draw_shadow(f, rect, area);
    f.render_widget(Clear, rect);
    let items: Vec<ListItem> = sel
        .sources
        .iter()
        .map(|(src, label)| {
            let mark = if sel.selected.contains(src) {
                "[x]"
            } else {
                "[ ]"
            };
            ListItem::new(Line::from(vec![
                Span::styled(
                    format!(" {mark}  "),
                    surface.fg(SURFACE_ACCENT).add_modifier(Modifier::BOLD),
                ),
                Span::styled(label.clone(), surface),
            ]))
        })
        .collect();
    let mut list_state = ListState::default();
    list_state.select(Some(sel.cursor));
    let title = format!(
        " Filter sources ({}/{}) ",
        sel.selected.len(),
        sel.sources.len()
    );
    let list = List::new(items)
        .block(dialog_block(title))
        .highlight_style(Style::default().add_modifier(Modifier::REVERSED))
        .highlight_symbol(" ▶ ")
        .highlight_spacing(HighlightSpacing::Always);
    f.render_stateful_widget(list, rect, &mut list_state);
    // Overlay a footer hint on the bottom-most row.
    if rect.height >= 2 {
        let hint_area = Rect {
            x: rect.x + 1,
            y: rect.y + rect.height - 1,
            width: rect.width.saturating_sub(2),
            height: 1,
        };
        let hint = Paragraph::new(" Space toggle | a toggle-all | Enter apply | Esc cancel ")
            .style(surface.fg(SURFACE_ACCENT));
        f.render_widget(hint, hint_area);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tui::state::Status;
    use ratatui::Terminal;
    use ratatui::backend::TestBackend;
    use serde_json::json;
    use std::collections::HashMap;

    /// A small populated state: one device (src 10) producing two PGNs,
    /// enough to exercise every screen's row rendering.
    fn seeded_state() -> AppState {
        let mut s = AppState::new(Status::new_log("cap.log".into()), HashMap::new());
        s.status.snapshot_loaded = true;
        s.upsert(
            60928,
            10,
            None,
            "ISO Address Claim".into(),
            json!({"pgn": 60928, "fields": {"Manufacturer Code": "Furuno"}}),
        );
        s.upsert(
            129025,
            10,
            None,
            "Position, Rapid Update".into(),
            json!({"pgn": 129025, "fields": {"Latitude": 53.1}}),
        );
        s.upsert(
            129025,
            10,
            None,
            "Position, Rapid Update".into(),
            json!({"pgn": 129025, "fields": {"Latitude": 53.2}}),
        );
        s
    }

    #[test]
    fn advertised_tx_pgns_become_overridable_rows() {
        let mut s = AppState::new(Status::new_log("cap.log".into()), HashMap::new());
        // A Furuno device that claims an address (so its NAME manufacturer
        // is known, disambiguating proprietary PGN variants) ...
        s.upsert(
            60928,
            52,
            None,
            "ISO Address Claim".into(),
            json!({"pgn":60928,"fields":{"Manufacturer Code":1855,"Unique Number":42}}),
        );
        // ... emits one live PGN ...
        s.upsert(
            127250,
            52,
            None,
            "Vessel Heading".into(),
            json!({"pgn":127250,"fields":{"Heading":1.0}}),
        );
        // ... and advertises a Transmit list (126464) naming a proprietary
        // Furuno PGN (130842) it isn't currently sending, plus the live one.
        s.upsert(
            126464,
            52,
            None,
            "PGN List".into(),
            json!({"pgn":126464,"fields":{"Function Code":0,
                "list":[{"PGN":130842},{"PGN":127250}]}}),
        );

        let rows = ui_app().detail_rows(52, &s);
        // The advertised 130842 appears as a count==0 row with a non-null
        // synthetic line; the already-live 127250 is not duplicated.
        let advertised: Vec<_> = rows.iter().filter(|e| e.count == 0).collect();
        assert_eq!(advertised.len(), 1);
        let row = advertised[0];
        assert_eq!(row.pgn, 130842);
        assert!(
            !row.line.is_null(),
            "advertised rows carry a synthetic line"
        );
        assert_eq!(
            rows.iter().filter(|e| e.pgn == 127250).count(),
            1,
            "the live PGN is not re-advertised"
        );
        // Its proprietary codes resolve from the schema (Furuno / marine)
        // so an override can target it even though it was never decoded.
        assert_eq!(proprietary_codes(row), (Some(1855), Some(4)));
    }

    #[test]
    fn advertised_variant_picks_the_device_manufacturer() {
        // 130842 has several manufacturer variants; the Furuno NAME selects
        // Furuno (1855) / marine (4), not whichever variant is listed first.
        let v = advertised_variant(130842, Some(1855)).expect("furuno variant");
        assert_eq!(variant_manufacturer(v), Some(1855));
        assert_eq!(variant_industry(v), Some(4));
        assert!(v.description.contains("Furuno"));
        // An unknown manufacturer must not guess a variant.
        assert!(advertised_variant(130842, None).is_none());
        // A standard PGN resolves to its single definition regardless.
        assert!(advertised_variant(127250, None).is_some());
    }

    /// An `App` in the settled post-startup state — most tests want the
    /// real UI, not the splash or the "Connecting…" overlay that
    /// `App::new` starts on.
    fn ui_app() -> App {
        let mut app = App::new();
        app.splash = None;
        app.connecting_dismissed = true;
        app
    }

    /// Render at a specific splash frame on a `w`×`h` backend.
    fn splash_buffer(frame: u16, w: u16, h: u16) -> String {
        let state = seeded_state();
        let mut app = App::new();
        app.splash = Some(frame);
        let backend = TestBackend::new(w, h);
        let mut term = Terminal::new(backend).unwrap();
        term.draw(|f| render(f, &mut app, &state)).unwrap();
        buffer_text(&term)
    }

    fn buffer_text(term: &Terminal<TestBackend>) -> String {
        let buf = term.backend().buffer().clone();
        let mut out = String::new();
        for y in 0..buf.area.height {
            for x in 0..buf.area.width {
                out.push_str(buf.cell((x, y)).map(|c| c.symbol()).unwrap_or(" "));
            }
            out.push('\n');
        }
        out
    }

    fn draw_to_buffer(app: &mut App, state: &AppState) -> String {
        let backend = TestBackend::new(120, 30);
        let mut term = Terminal::new(backend).unwrap();
        term.draw(|f| render(f, app, state)).unwrap();
        buffer_text(&term)
    }

    #[test]
    fn timeline_windows_only_the_viewport() {
        let mut state = seeded_state();
        for i in 0..5000 {
            state.upsert(
                127250,
                10,
                None,
                "Heading".into(),
                json!({"pgn": 127250, "i": i}),
            );
        }
        let app = ui_app();
        let total = app.visible_history_len(&state);
        assert!(total >= 5000);
        // A window returns at most `len` indices, starting at `offset`.
        let w = app.visible_window(&state, 100, 30);
        assert_eq!(w.len(), 30);
        assert_eq!(w[0], 100);
        // A window past the tail is truncated, not padded.
        let tail = app.visible_window(&state, total - 5, 30);
        assert_eq!(tail.len(), 5);
    }

    #[test]
    fn timeline_scrolls_to_keep_selection_visible() {
        let mut state = seeded_state();
        for i in 0..1000 {
            state.upsert(
                127250,
                10,
                None,
                "Heading".into(),
                json!({"pgn": 127250, "i": i}),
            );
        }
        let mut app = ui_app();
        app.screen = Screen::TimeView;
        let total = app.visible_history_len(&state);
        // Jump the cursor near the end and render; the offset must follow
        // so the selected row is on-screen.
        app.time_state.select(Some(total - 1));
        let _ = draw_to_buffer(&mut app, &state);
        assert!(app.time_offset > 0, "offset should track the selection");
        assert!(app.time_offset < total, "offset stays within bounds");
        // Selection back to the top rewinds the offset on the next draw.
        app.time_state.select(Some(0));
        let _ = draw_to_buffer(&mut app, &state);
        assert_eq!(app.time_offset, 0);
    }

    #[test]
    fn every_screen_renders_without_panic() {
        let state = seeded_state();
        for screen in [
            Screen::Devices,
            Screen::TimeView,
            Screen::PgnTop,
            Screen::Nmea0183,
            Screen::Overrides,
            Screen::DeviceDetail { src: 10 },
        ] {
            let mut app = ui_app();
            app.screen = screen;
            let _ = draw_to_buffer(&mut app, &state);
        }
    }

    #[test]
    fn splash_animates_and_ends() {
        // Early frame: window still zooming, content not yet shown — must
        // not panic on the tiny interior.
        let _ = splash_buffer(0, 120, 30);
        // Late frame on a roomy screen shows the banner + boot block.
        let out = splash_buffer(SPLASH_GROW, 120, 30);
        assert!(out.contains("READY."), "C64 boot line missing");
        assert!(out.contains("KEES VERRUIJT"), "copyright missing");
        // Tail phase shows the animated loading verb (log mode → LOADING).
        let out = splash_buffer(SPLASH_LOAD_START + 2, 120, 30);
        assert!(out.contains("LOADING"), "loading tail missing");
        // Frame advance clears itself once past the last frame.
        let mut app = App::new();
        app.splash = Some(SPLASH_FRAMES - 1);
        let state = seeded_state();
        let backend = TestBackend::new(120, 30);
        let mut term = Terminal::new(backend).unwrap();
        term.draw(|f| render(f, &mut app, &state)).unwrap();
        assert!(app.splash.is_none(), "splash should end after last frame");
    }

    #[test]
    fn menu_bar_shows_titles_and_every_dropdown_renders() {
        let state = seeded_state();
        let mut app = ui_app();
        // Bar closed: the titles are painted on the top row.
        let out = draw_to_buffer(&mut app, &state);
        for title in ["File", "View", "Device", "Search", "Help"] {
            assert!(out.contains(title), "menu title {title} missing from bar");
        }
        // Open each menu in turn — exercises the drop-down + shadow
        // placement / clamping for every column.
        let n = app.build_menus(&state).len();
        for i in 0..n {
            app.menu.open = Some(i);
            app.menu.item = app.build_menus(&state)[i].first_item();
            let _ = draw_to_buffer(&mut app, &state);
        }
    }

    #[test]
    fn pgn_load_view_lists_the_busy_pgn() {
        let state = seeded_state();
        let mut app = ui_app();
        app.screen = Screen::PgnTop;
        let out = draw_to_buffer(&mut app, &state);
        assert!(
            out.contains("129025"),
            "PGN 129025 should appear in load view"
        );
        assert!(out.contains("PGN Load"), "load view title missing");
    }

    #[test]
    fn help_modal_renders() {
        let state = seeded_state();
        let mut app = ui_app();
        app.help_visible = true;
        let out = draw_to_buffer(&mut app, &state);
        assert!(out.contains("Help"), "help title missing");
        assert!(out.contains("F10"), "help should document F10");
    }

    #[test]
    fn connect_dialog_renders() {
        let state = seeded_state();
        let mut app = ui_app();
        app.open_connect_modal(&state);
        let out = draw_to_buffer(&mut app, &state);
        assert!(out.contains("Connect"), "Connect dialog title missing");
    }

    #[test]
    fn file_browser_renders_and_lists_entries() {
        let state = seeded_state();
        let mut app = ui_app();
        app.open_file_browser();
        let out = draw_to_buffer(&mut app, &state);
        assert!(out.contains("Load"), "browser title missing");
        // The parent entry is always present in a browsable directory.
        assert!(out.contains(".."), "parent entry missing");
    }

    #[test]
    fn connect_dialog_parses_host_and_ports() {
        let mut app = ui_app();
        app.io_modal = Some(IoModal {
            input: "pi.local 1234 5678".into(),
        });
        app.handle_io_modal_key(KeyEvent::from(KeyCode::Enter));
        match app.pending_command {
            Some(PendingCommand::Connect {
                host,
                snapshot_port,
                stream_port,
            }) => {
                assert_eq!(host, "pi.local");
                assert_eq!(snapshot_port, 1234);
                assert_eq!(stream_port, 5678);
            }
            _ => panic!("expected a Connect command"),
        }
    }

    #[test]
    fn save_browser_toggles_format_types_name_and_saves() {
        let mut app = ui_app();
        app.open_save_browser();
        let fb = app.file_browser.as_ref().unwrap();
        assert!(fb.save && fb.filename.ends_with(".json"));
        // Tab toggles to Raw and rewrites the extension.
        app.handle_file_browser_key(KeyEvent::from(KeyCode::Tab));
        let fb = app.file_browser.as_ref().unwrap();
        assert_eq!(fb.format, SaveFormat::Raw);
        assert!(fb.filename.ends_with(".raw"));
        // Typing appends to the filename.
        app.handle_file_browser_key(KeyEvent::from(KeyCode::Char('X')));
        assert!(app.file_browser.as_ref().unwrap().filename.ends_with('X'));
        // Enter confirms into dir/filename with the chosen format.
        let expected = app.file_browser.as_ref().unwrap().save_path();
        app.handle_file_browser_key(KeyEvent::from(KeyCode::Enter));
        match app.pending_command {
            Some(PendingCommand::Save { path, format }) => {
                assert_eq!(path, expected);
                assert_eq!(format, SaveFormat::Raw);
            }
            _ => panic!("expected a Save command"),
        }
    }

    #[test]
    fn on_request_pgns_are_flagged() {
        // Schema-irregular (on request / on event).
        assert!(pgn_on_request(60928)); // ISO Address Claim
        assert!(pgn_on_request(126996)); // Product Information
        assert!(pgn_on_request(126998)); // Configuration Information
        // Regular, fixed-interval PGNs.
        assert!(!pgn_on_request(126993)); // Heartbeat (60 s)
        assert!(!pgn_on_request(129025)); // Position, Rapid Update
    }

    #[test]
    fn robust_interval_ignores_merge_gaps() {
        // A 60 s heartbeat with a big idle gap in the middle (as in a
        // merged capture): the first→last average is skewed long, but
        // the median of recent gaps recovers ~60 s.
        let mut s = seeded_state();
        let ts =
            |ms: i64| json!({"pgn": 126993, "timestamp": canboat_core::format_iso_ms(ms as u64)});
        // 10 frames at 60 s, then a 6-hour gap, then 10 more at 60 s.
        let mut t = 0i64;
        for _ in 0..10 {
            s.upsert(126993, 20, None, "Heartbeat".into(), ts(t));
            t += 60_000;
        }
        t += 6 * 3_600_000; // idle gap
        for _ in 0..10 {
            s.upsert(126993, 20, None, "Heartbeat".into(), ts(t));
            t += 60_000;
        }
        let e = s.entries.get(&(126993, 20, None)).unwrap();
        // First→last average is badly inflated by the gap...
        assert!(e.interval().unwrap().as_secs() > 1_000);
        // ...but the robust median lands on the true 60 s cadence.
        let robust = robust_interval(e, &s).unwrap().as_secs();
        assert!((59..=61).contains(&robust), "got {robust}s");
    }

    #[test]
    fn human_size_is_compact() {
        assert_eq!(human_size(0), "0B");
        assert_eq!(human_size(512), "512B");
        assert_eq!(human_size(1024), "1.0K");
        assert_eq!(human_size(1536), "1.5K");
        assert_eq!(human_size(734_003_200), "700M"); // 700 MiB
    }

    #[test]
    fn mtime_formats_as_local_ymd_hm() {
        use std::time::{Duration, UNIX_EPOCH};
        // Local time is machine-dependent, so assert the shape
        // (`YYYY-MM-DD HH:MM`) rather than an exact value.
        let s = format_mtime(UNIX_EPOCH + Duration::from_secs(1_609_459_200));
        assert_eq!(s.len(), 16, "{s:?}");
        let b = s.as_bytes();
        assert_eq!((b[4], b[7], b[10], b[13]), (b'-', b'-', b' ', b':'));
        assert_eq!(s.chars().filter(|c| c.is_ascii_digit()).count(), 12);
    }

    #[test]
    fn open_dialog_swallows_menu_and_help_keys() {
        let state = seeded_state();
        let (writer, _rx) = crate::tui::client::make_writer();
        // With the file browser up, F10 / Alt+F / F1 must not open the
        // menu or Help behind it.
        let mut app = ui_app();
        app.open_file_browser();
        app.handle_key(KeyEvent::from(KeyCode::F(10)), &state, &writer);
        assert!(!app.menu.is_open(), "F10 leaked to the menu behind browser");
        app.handle_key(
            KeyEvent::new(KeyCode::Char('f'), KeyModifiers::ALT),
            &state,
            &writer,
        );
        assert!(
            !app.menu.is_open(),
            "Alt+F leaked to the menu behind browser"
        );
        app.handle_key(KeyEvent::from(KeyCode::F(1)), &state, &writer);
        assert!(!app.help_visible, "F1 leaked to Help behind browser");
        assert!(app.file_browser.is_some(), "browser stayed open");
    }

    #[test]
    fn browser_remembers_directory_across_opens() {
        let root = std::env::temp_dir();
        let mut app = ui_app();
        app.browse_dir = Some(root.clone());
        // Load browser resumes the remembered directory.
        app.open_file_browser();
        assert_eq!(app.file_browser.as_ref().unwrap().cwd, root);
        // Esc closes but keeps the directory memory.
        app.handle_file_browser_key(KeyEvent::from(KeyCode::Esc));
        assert!(app.file_browser.is_none());
        assert_eq!(app.browse_dir.as_ref(), Some(&root));
        // A subsequent Save browser resumes there too.
        app.open_save_browser();
        assert_eq!(app.file_browser.as_ref().unwrap().cwd, root);
        // Navigating up updates the remembered directory.
        app.handle_file_browser_key(KeyEvent::from(KeyCode::Left));
        let cwd = app.file_browser.as_ref().unwrap().cwd.clone();
        assert_eq!(app.browse_dir.as_ref(), Some(&cwd));
        assert_ne!(cwd, root);
    }

    #[test]
    fn prompt_load_opens_file_browser() {
        let mut app = ui_app();
        app.prompt_load();
        assert!(
            app.file_browser.is_some(),
            "startup should open the browser"
        );
        let out = draw_to_buffer(&mut app, &seeded_state());
        assert!(out.contains("Load"), "browser should be visible");
    }

    #[test]
    fn connect_dialog_defaults_ports_when_omitted() {
        let mut app = ui_app();
        app.io_modal = Some(IoModal {
            input: "boat".into(),
        });
        app.handle_io_modal_key(KeyEvent::from(KeyCode::Enter));
        match app.pending_command {
            Some(PendingCommand::Connect {
                host,
                snapshot_port,
                stream_port,
            }) => {
                assert_eq!(host, "boat");
                assert_eq!(snapshot_port, 2597);
                assert_eq!(stream_port, 2598);
            }
            _ => panic!("expected a Connect command"),
        }
    }
}
