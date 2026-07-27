// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Turbo Pascal / EDIT.COM-style menu bar, pull-down menus, and the
//! shared retro colour palette.
//!
//! The bar lives on the top row; pressing `F10` (or `Alt`+a title's
//! highlighted letter) drops down the corresponding menu. Left/Right
//! switch menus, Up/Down move the highlight, Enter (or a hotkey
//! letter) fires the item's [`MenuAction`], Esc closes. Every existing
//! single-key shortcut keeps working as an accelerator — the menu is
//! purely additive discovery.
//!
//! Rendering is deliberately dumb: the caller rebuilds the [`Menu`]
//! list every frame (cheaply) with the right enabled/disabled state
//! for the current screen, hands it here, and this module paints the
//! bar plus, if one is open, the drop-down with its drop shadow.

use ratatui::layout::Rect;
use ratatui::style::{Color, Modifier, Style};
use ratatui::text::{Line, Span};
use ratatui::widgets::{Block, BorderType, Borders, Clear, Paragraph};

// ── Palette ────────────────────────────────────────────────────────
// Borland Turbo Pascal 7 / DOS EDIT.COM colours, mapped onto the 16
// ANSI slots. `Color::Gray` is ANSI "white" (light grey) and
// `Color::DarkGray` is ANSI "bright black" — so Gray reads as the
// classic light-grey menu/dialog surface and DarkGray as the shadow /
// disabled ink.

/// The blue desktop behind every window. Pinned to the classic DOS
/// `#0000AA` via RGB rather than the ANSI `Blue` slot, which many
/// terminal palettes render as a washed-out light blue.
pub const DESKTOP_BG: Color = Color::Rgb(0, 0, 0xAA);
/// Content window background (the blue "editor" panel).
pub const PANEL_BG: Color = Color::Rgb(0, 0, 0xAA);
/// Content window text.
pub const PANEL_FG: Color = Color::Gray;
/// Content window double-line border.
pub const PANEL_BORDER: Color = Color::Cyan;
/// Content window title text.
pub const PANEL_TITLE: Color = Color::White;

/// Grey surface shared by the menu bar, drop-downs, dialogs, and the
/// bottom status line. A dark charcoal with near-white ink — high
/// contrast, and darker than the ANSI `Gray` slot (which renders as a
/// washed-out near-white).
pub const SURFACE_BG: Color = Color::Rgb(0x40, 0x40, 0x40);
pub const SURFACE_FG: Color = Color::Rgb(0xf2, 0xf2, 0xf2);
/// The highlighted hot-key letter in a title / item label — a bright
/// red that pops against the charcoal surface.
pub const HOTKEY_FG: Color = Color::Rgb(0xff, 0x6b, 0x6b);
/// A disabled menu item — dim but still legible against [`SURFACE_BG`].
pub const DISABLED_FG: Color = Color::Rgb(0x8c, 0x8c, 0x8c);
/// Accent for dialog titles / hints on the grey surface (light blue).
pub const SURFACE_ACCENT: Color = Color::Rgb(0x66, 0xcc, 0xff);
/// The selection bar inside a drop-down / the open menu's bar title.
pub const SELECT_BG: Color = Color::Rgb(0x00, 0x87, 0x00);
pub const SELECT_FG: Color = Color::Rgb(0xff, 0xff, 0xff);
/// Drop shadow cast by drop-downs and dialogs.
pub const SHADOW_BG: Color = Color::Black;

/// What a menu item does when fired. Interpreted by the UI dispatcher
/// against the current screen / selection; several actions are only
/// meaningful on a particular screen (and their menu items are marked
/// disabled elsewhere).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MenuAction {
    Quit,
    FileConnect,
    FileLoad,
    FileSave,
    ViewDevices,
    ViewTimeline,
    ViewPgnLoad,
    ViewNmea0183,
    ViewOverrides,
    Back,
    OpenSelected,
    IsoRequest126464,
    OverrideInterval,
    SearchFind,
    SearchNext,
    SearchPrev,
    FilterPgns,
    FilterSources,
    Help,
    About,
}

/// One entry in a pull-down: either an actionable item or a separator
/// rule.
pub enum MenuEntry {
    Item(MenuItem),
    Separator,
}

pub struct MenuItem {
    pub label: String,
    /// The highlighted letter; matched case-insensitively while the
    /// menu is open. `None` for items with no accelerator letter.
    pub hotkey: Option<char>,
    /// Right-aligned accelerator hint (e.g. `"t"`, `"Alt-X"`). Empty
    /// when the item has none.
    pub accel: String,
    pub action: MenuAction,
    /// Greyed + non-firing when `false`.
    pub enabled: bool,
}

impl MenuItem {
    // A factory returning the wrapping `MenuEntry`, not `Self` — the
    // call sites read cleanly as `MenuItem::new(...)` in the menu tree.
    #[allow(clippy::new_ret_no_self)]
    pub fn new(label: &str, hotkey: Option<char>, accel: &str, action: MenuAction) -> MenuEntry {
        MenuEntry::Item(MenuItem {
            label: label.to_string(),
            hotkey,
            accel: accel.to_string(),
            action,
            enabled: true,
        })
    }

    /// Same as [`MenuItem::new`] but with an explicit enabled flag —
    /// used for context-sensitive items.
    pub fn maybe(
        label: &str,
        hotkey: Option<char>,
        accel: &str,
        action: MenuAction,
        enabled: bool,
    ) -> MenuEntry {
        let mut e = Self::new(label, hotkey, accel, action);
        if let MenuEntry::Item(item) = &mut e {
            item.enabled = enabled;
        }
        e
    }
}

pub struct Menu {
    pub title: &'static str,
    /// Highlighted title letter — matched against `Alt`+letter to open
    /// this menu directly.
    pub hotkey: char,
    pub entries: Vec<MenuEntry>,
}

impl Menu {
    /// Index of the first actionable (non-separator) entry, used to
    /// place the highlight when the menu opens.
    pub fn first_item(&self) -> usize {
        self.entries
            .iter()
            .position(|e| matches!(e, MenuEntry::Item(_)))
            .unwrap_or(0)
    }

    /// Step the highlight by `delta`, wrapping and skipping
    /// separators. No-op if the menu has no actionable entries.
    pub fn step(&self, from: usize, delta: i32) -> usize {
        let n = self.entries.len();
        if n == 0 {
            return 0;
        }
        let mut i = from as i32;
        for _ in 0..n {
            i = (i + delta).rem_euclid(n as i32);
            if matches!(self.entries[i as usize], MenuEntry::Item(_)) {
                return i as usize;
            }
        }
        from
    }
}

/// Which pull-down is open (if any) and where the highlight sits.
pub struct MenuBar {
    /// `Some(i)` when menu `i` is dropped down; `None` when the bar is
    /// idle.
    pub open: Option<usize>,
    pub item: usize,
}

impl MenuBar {
    pub fn new() -> Self {
        Self {
            open: None,
            item: 0,
        }
    }

    pub fn is_open(&self) -> bool {
        self.open.is_some()
    }
}

/// Left column (relative to `bar_area.x`) of each menu title, so a
/// drop-down can be placed flush under its title. Titles render as
/// `" Title "` with no extra gap — the padding spaces do the
/// separating.
fn title_columns(bar_area: Rect, menus: &[Menu]) -> Vec<u16> {
    let mut cols = Vec::with_capacity(menus.len());
    let mut x = bar_area.x;
    for m in menus {
        cols.push(x);
        x += m.title.chars().count() as u16 + 2;
    }
    cols
}

/// Split `" Title "` into spans with the first case-insensitive match
/// of `hotkey` painted in `hotkey_style`, the rest in `base`.
fn label_spans(
    padded: &str,
    hotkey: Option<char>,
    base: Style,
    hotkey_style: Style,
) -> Vec<Span<'static>> {
    let Some(hk) = hotkey else {
        return vec![Span::styled(padded.to_string(), base)];
    };
    let lower = hk.to_ascii_lowercase();
    let mut pos = None;
    for (i, c) in padded.char_indices() {
        if c.to_ascii_lowercase() == lower {
            pos = Some(i);
            break;
        }
    }
    match pos {
        None => vec![Span::styled(padded.to_string(), base)],
        Some(i) => {
            let (before, rest) = padded.split_at(i);
            let mut chars = rest.chars();
            let hc = chars.next().unwrap();
            let after: String = chars.collect();
            let mut spans = Vec::new();
            if !before.is_empty() {
                spans.push(Span::styled(before.to_string(), base));
            }
            spans.push(Span::styled(hc.to_string(), hotkey_style));
            if !after.is_empty() {
                spans.push(Span::styled(after, base));
            }
            spans
        }
    }
}

/// Paint the top menu bar across `area` (a single row).
pub fn draw_bar(f: &mut ratatui::Frame<'_>, area: Rect, menus: &[Menu], mb: &MenuBar) {
    let base = Style::default().bg(SURFACE_BG).fg(SURFACE_FG);
    // Fill the whole row first so titles sit on an unbroken grey strip.
    f.render_widget(Block::default().style(base), area);
    let mut spans = Vec::new();
    for (i, m) in menus.iter().enumerate() {
        let open = mb.open == Some(i);
        let (item_base, hk_style) = if open {
            let b = Style::default().bg(SELECT_BG).fg(SELECT_FG);
            (b, b.add_modifier(Modifier::BOLD | Modifier::UNDERLINED))
        } else {
            (base, base.fg(HOTKEY_FG).add_modifier(Modifier::BOLD))
        };
        let padded = format!(" {} ", m.title);
        spans.extend(label_spans(&padded, Some(m.hotkey), item_base, hk_style));
    }
    f.render_widget(Paragraph::new(Line::from(spans)).style(base), area);
}

/// Paint the open pull-down (if any) plus its shadow. `bar_area` is
/// the menu-bar row; `screen` is the whole frame, used to clamp the
/// box onto the terminal.
pub fn draw_dropdown(
    f: &mut ratatui::Frame<'_>,
    bar_area: Rect,
    screen: Rect,
    menus: &[Menu],
    mb: &MenuBar,
) {
    let Some(open) = mb.open else {
        return;
    };
    let Some(menu) = menus.get(open) else {
        return;
    };
    let cols = title_columns(bar_area, menus);
    let left = cols.get(open).copied().unwrap_or(bar_area.x);

    // Inner width: widest "label ….. accel" pair, min a sane floor.
    let inner_w = menu
        .entries
        .iter()
        .map(|e| match e {
            MenuEntry::Separator => 0,
            MenuEntry::Item(it) => {
                it.label.chars().count()
                    + if it.accel.is_empty() {
                        0
                    } else {
                        2 + it.accel.chars().count()
                    }
            }
        })
        .max()
        .unwrap_or(0)
        .max(8) as u16;
    let width = inner_w + 4; // 2 borders + 1 pad each side
    let height = menu.entries.len() as u16 + 2; // 2 borders

    // Clamp horizontally so the box + shadow stay on screen.
    let max_x = screen.x + screen.width.saturating_sub(width + 1);
    let x = left.min(max_x).max(screen.x);
    let y = bar_area.y + 1;
    let rect = Rect {
        x,
        y,
        width,
        height: height.min(screen.height.saturating_sub(y - screen.y).saturating_sub(1)),
    };

    draw_shadow(f, rect, screen);
    f.render_widget(Clear, rect);

    let surface = Style::default().bg(SURFACE_BG).fg(SURFACE_FG);
    let mut lines: Vec<Line> = Vec::with_capacity(menu.entries.len());
    for (i, entry) in menu.entries.iter().enumerate() {
        match entry {
            MenuEntry::Separator => {
                lines.push(Line::from(Span::styled(
                    "─".repeat(inner_w as usize),
                    surface.fg(DISABLED_FG),
                )));
            }
            MenuEntry::Item(it) => {
                let selected = i == mb.item;
                let (base, hk_style) = if selected {
                    let b = Style::default().bg(SELECT_BG).fg(SELECT_FG);
                    (b, b.add_modifier(Modifier::BOLD | Modifier::UNDERLINED))
                } else if !it.enabled {
                    let b = surface.fg(DISABLED_FG);
                    (b, b)
                } else {
                    (surface, surface.fg(HOTKEY_FG).add_modifier(Modifier::BOLD))
                };
                // Compose "label<pad>accel" to the full inner width.
                let label_w = it.label.chars().count();
                let accel_w = it.accel.chars().count();
                let pad = (inner_w as usize).saturating_sub(label_w + accel_w).max(1);
                let mut spans = label_spans(&it.label, it.hotkey, base, hk_style);
                spans.push(Span::styled(" ".repeat(pad), base));
                if !it.accel.is_empty() {
                    let accel_style = if selected { base } else { base.fg(DISABLED_FG) };
                    spans.push(Span::styled(it.accel.clone(), accel_style));
                }
                lines.push(Line::from(spans));
            }
        }
    }
    let block = Block::default()
        .borders(Borders::ALL)
        .border_type(BorderType::Double)
        .border_style(surface)
        .style(surface);
    f.render_widget(Paragraph::new(lines).block(block), rect);
}

/// Cast an L-shaped drop shadow along the right and bottom edges of
/// `rect`, clamped to `screen`.
pub fn draw_shadow(f: &mut ratatui::Frame<'_>, rect: Rect, screen: Rect) {
    let shadow = Block::default().style(Style::default().bg(SHADOW_BG));
    // Right edge, shifted one row down.
    let rx = rect.x + rect.width;
    if rx < screen.x + screen.width {
        let ry = rect.y + 1;
        let rh = rect.height.min(screen.y + screen.height - ry);
        if rh > 0 {
            f.render_widget(
                Clear,
                Rect {
                    x: rx,
                    y: ry,
                    width: 1,
                    height: rh,
                },
            );
            f.render_widget(
                shadow.clone(),
                Rect {
                    x: rx,
                    y: ry,
                    width: 1,
                    height: rh,
                },
            );
        }
    }
    // Bottom edge, shifted one column right.
    let by = rect.y + rect.height;
    if by < screen.y + screen.height {
        let bx = rect.x + 1;
        let bw = rect.width.min(screen.x + screen.width - bx);
        if bw > 0 {
            f.render_widget(
                Clear,
                Rect {
                    x: bx,
                    y: by,
                    width: bw,
                    height: 1,
                },
            );
            f.render_widget(
                shadow,
                Rect {
                    x: bx,
                    y: by,
                    width: bw,
                    height: 1,
                },
            );
        }
    }
}
