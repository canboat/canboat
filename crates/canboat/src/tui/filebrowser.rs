// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! A minimal directory/file browser backing the File ▸ Load dialog.
//!
//! Pure state + filesystem logic (no ratatui) so it can be unit-tested
//! headless; [`crate::tui::ui`] owns the rendering and key wiring. Entries
//! are listed parent-first, then sub-directories, then files, each
//! group sorted case-insensitively.

use std::path::PathBuf;
use std::time::SystemTime;

use crate::tui::client::SaveFormat;

/// One row in the browser: a directory (including the `..` parent) or a
/// selectable file.
pub struct FileEntry {
    /// Display name (`..` for the parent, otherwise the file name).
    pub name: String,
    /// Absolute path this row points at.
    pub path: PathBuf,
    pub is_dir: bool,
    /// Size in bytes (files only; `0` for directories).
    pub size: u64,
    /// Last-modified time, if the platform / filesystem reports one.
    pub modified: Option<SystemTime>,
}

pub struct FileBrowser {
    /// Directory currently listed.
    pub cwd: PathBuf,
    pub entries: Vec<FileEntry>,
    pub selected: usize,
    /// Set when the current directory can't be read (permissions, gone);
    /// the listing still shows `..` so the user can escape.
    pub error: Option<String>,
    /// `true` for the Save browser: the dialog also carries an editable
    /// [`Self::filename`] + [`Self::format`] and confirms into
    /// `cwd/filename`. `false` for Load (pick an existing file).
    pub save: bool,
    /// Save mode: the editable output file name (empty in Load mode).
    pub filename: String,
    /// Save mode: the chosen output format.
    pub format: SaveFormat,
}

impl FileBrowser {
    /// Open a Load browser rooted at `start` (falling back to `/` if
    /// that isn't usable) and read its contents.
    pub fn open(start: PathBuf) -> Self {
        let cwd = if start.is_dir() {
            start
        } else {
            start
                .parent()
                .map(|p| p.to_path_buf())
                .unwrap_or_else(|| PathBuf::from("/"))
        };
        let mut b = Self {
            cwd,
            entries: Vec::new(),
            selected: 0,
            error: None,
            save: false,
            filename: String::new(),
            format: SaveFormat::Analysed,
        };
        b.refresh();
        b
    }

    /// Open a Save browser: navigate to a directory, edit `default_name`,
    /// pick `format`, then confirm into `cwd/filename`.
    pub fn open_save(start: PathBuf, default_name: String, format: SaveFormat) -> Self {
        let mut b = Self::open(start);
        b.save = true;
        b.filename = default_name;
        b.format = format;
        b
    }

    /// Save mode: the path a confirm would write to.
    pub fn save_path(&self) -> PathBuf {
        self.cwd.join(&self.filename)
    }

    /// Re-read [`Self::cwd`], rebuilding [`Self::entries`] (parent, then
    /// dirs, then files). Clamps the selection.
    pub fn refresh(&mut self) {
        self.error = None;
        let mut dirs: Vec<FileEntry> = Vec::new();
        let mut files: Vec<FileEntry> = Vec::new();
        match std::fs::read_dir(&self.cwd) {
            Ok(rd) => {
                for entry in rd.flatten() {
                    let path = entry.path();
                    let name = entry.file_name().to_string_lossy().into_owned();
                    // Follow symlinks (matches what a user navigating
                    // expects); a broken link falls back to a size-0 file.
                    let meta = std::fs::metadata(&path).ok();
                    let is_dir = meta.as_ref().is_some_and(|m| m.is_dir());
                    let modified = meta.as_ref().and_then(|m| m.modified().ok());
                    let size = meta.as_ref().map(|m| m.len()).unwrap_or(0);
                    let fe = FileEntry {
                        name,
                        path,
                        is_dir,
                        size,
                        modified,
                    };
                    if is_dir {
                        dirs.push(fe)
                    } else {
                        files.push(fe)
                    }
                }
            }
            Err(e) => self.error = Some(e.to_string()),
        }
        let key = |e: &FileEntry| e.name.to_lowercase();
        dirs.sort_by_key(key);
        files.sort_by_key(key);

        let mut entries = Vec::with_capacity(dirs.len() + files.len() + 1);
        if let Some(parent) = self.cwd.parent() {
            entries.push(FileEntry {
                name: "..".to_string(),
                path: parent.to_path_buf(),
                is_dir: true,
                size: 0,
                modified: None,
            });
        }
        entries.append(&mut dirs);
        entries.append(&mut files);
        self.entries = entries;
        if self.selected >= self.entries.len() {
            self.selected = self.entries.len().saturating_sub(1);
        }
    }

    /// Move the selection by `delta`, wrapping.
    pub fn move_selection(&mut self, delta: i32) {
        let n = self.entries.len();
        if n == 0 {
            return;
        }
        let cur = self.selected as i32;
        self.selected = (cur + delta).rem_euclid(n as i32) as usize;
    }

    /// Activate the current row. Descends into a directory (re-reading
    /// the listing) and returns `None`; returns `Some(path)` when the
    /// row is a file the caller should load.
    pub fn activate(&mut self) -> Option<PathBuf> {
        let entry = self.entries.get(self.selected)?;
        if entry.is_dir {
            self.cwd = entry.path.clone();
            self.selected = 0;
            self.refresh();
            None
        } else {
            Some(entry.path.clone())
        }
    }

    /// Ascend to the parent directory, if any, landing the cursor on the
    /// directory we came out of.
    pub fn go_parent(&mut self) {
        if let Some(parent) = self.cwd.parent() {
            let child = self
                .cwd
                .file_name()
                .map(|s| s.to_string_lossy().into_owned());
            self.cwd = parent.to_path_buf();
            self.selected = 0;
            self.refresh();
            if let Some(child) = child
                && let Some(pos) = self.entries.iter().position(|e| e.name == child)
            {
                self.selected = pos;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tmp(name: &str) -> PathBuf {
        let d = std::env::temp_dir().join(format!("cbtui-fb-{}-{name}", std::process::id()));
        let _ = std::fs::remove_dir_all(&d);
        std::fs::create_dir_all(&d).unwrap();
        d
    }

    #[test]
    fn lists_parent_dirs_then_files_sorted() {
        let root = tmp("list");
        std::fs::create_dir(root.join("zeta")).unwrap();
        std::fs::create_dir(root.join("alpha")).unwrap();
        std::fs::write(root.join("b.raw"), b"x").unwrap();
        std::fs::write(root.join("a.json"), b"x").unwrap();

        let b = FileBrowser::open(root.clone());
        let names: Vec<&str> = b.entries.iter().map(|e| e.name.as_str()).collect();
        // parent, dirs (sorted), then files (sorted).
        assert_eq!(names, vec!["..", "alpha", "zeta", "a.json", "b.raw"]);
        assert!(b.entries[1].is_dir && !b.entries[3].is_dir);
        let _ = std::fs::remove_dir_all(&root);
    }

    #[test]
    fn activate_directory_descends_file_returns_path() {
        let root = tmp("nav");
        std::fs::create_dir(root.join("sub")).unwrap();
        std::fs::write(root.join("sub").join("cap.raw"), b"x").unwrap();

        let mut b = FileBrowser::open(root.clone());
        // Select "sub" (index 1: after "..").
        b.selected = b.entries.iter().position(|e| e.name == "sub").unwrap();
        assert!(b.activate().is_none());
        assert_eq!(b.cwd, root.join("sub"));

        b.selected = b.entries.iter().position(|e| e.name == "cap.raw").unwrap();
        assert_eq!(b.activate(), Some(root.join("sub").join("cap.raw")));

        // Parent navigation climbs back out and lands on the child dir.
        b.go_parent();
        assert_eq!(b.cwd, root);
        assert_eq!(b.entries[b.selected].name, "sub");
        let _ = std::fs::remove_dir_all(&root);
    }

    #[test]
    fn move_selection_wraps() {
        let root = tmp("wrap");
        std::fs::write(root.join("only.raw"), b"x").unwrap();
        let mut b = FileBrowser::open(root.clone());
        let n = b.entries.len();
        b.move_selection(-1);
        assert_eq!(b.selected, n - 1);
        b.move_selection(1);
        assert_eq!(b.selected, 0);
        let _ = std::fs::remove_dir_all(&root);
    }
}
