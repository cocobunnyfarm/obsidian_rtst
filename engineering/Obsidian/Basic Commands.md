# Obsidian Crash Course for a Notion Pro

Welcome to Obsidian! Since you're coming from Notion, I'll highlight key differences and show you how Obsidian's philosophy differs while being just as powerful.

## 🧠 Core Mindset Shift
- **Notion**: Database-driven, structured, collaborative
- **Obsidian**: **Local-first**, **markdown-based**, **networked thought** tool
- Your files are just `.md` files in a folder on your computer
- No vendor lock-in - you own your notes forever

## 📝 Basic Editing (Markdown Fundamentals)

### Text Formatting
```
**Bold** or __Bold__
*Italic* or _Italic_
***Bold and Italic***
`inline code`
~~strikethrough~~
==highlight== (needs plugin)
```

### Headers
```
# H1
## H2
### H3
```

### Lists
```
- Bullet list
- Another item
  - Indented item

1. Numbered list
2. Second item
```

### Advanced Elements
```
> Blockquote
---
Horizontal rule

- [ ] Task list
- [x] Completed task
```

**Pro Tip**: Obsidian has a command palette (`Cmd/Ctrl+P`) - use it for everything!

## 🔗 Linking & Connections (This is Obsidian's Superpower)

### Basic Linking
```
[[Note Name]] - Link to another note
[[Note Name|Custom Display Text]]
```

### Internal Headers & Blocks
```
[[Note Name#Header]] - Link to specific header
[[Note Name#^block-id]] - Link to specific block
```

### Uncreated Notes
- `[[Non-existent Note]]` creates a clickable link that creates the note when clicked
- This encourages spontaneous linking without interruption

### Backlinks & Graph View
- **Every link is bidirectional** - if Note A links to Note B, Note B shows Note A in its backlinks
- **Graph View** shows visual relationships between notes

## 🗂️ Organization vs Notion

### Instead of Databases:
- **Tags**: `#tag` in your notes - use nested `#parent/child` tags
- **Frontmatter** (YAML at top of note):
```yaml
---
alias: ["Alternative Name"]
tags: [project, active]
due: 2024-01-15
---
```

### Instead of Nested Pages:
- Use **folders** for broad categories
- Use **links** for relationships (more flexible than hierarchy)

## 🔌 Essential Plugins to Enable (Core)

1. **Daily Notes** - Like Notion's template but for dates
2. **Templates** - Create note templates
3. **Outliner** - Better list handling
4. **Starred** - Bookmark important notes
5. **Outline** - Note navigation pane

## 🚀 Quickstart Workflow

1. **Create a Vault** (your notebook/workspace)
2. **Make your first note**: Just start typing!
3. **Link related notes** with `[[double brackets]]`
4. **Use the Graph View** to see connections
5. **Try the Local Graph** to see connections around current note

## ⚡ Power Features Notion Doesn't Have

### 1. **Canvas**
- Infinite whiteboard with connected notes/cards
- Perfect for brainstorming and mapping ideas

### 2. **Dataview** (Community Plugin)
```dataview
TABLE due, status
FROM #project
WHERE status != "completed"
SORT due ASC
```
- Query your notes like a database - this is the closest to Notion databases

### 3. **Quick Switcher** (`Cmd/Ctrl+O`)
- Instant note navigation

### 4. **Note Embedding**
```
![[Note Name]] - Embeds entire note
![[Note Name#Header]] - Embeds just a section
```

## 📁 File Management Tips
- Name notes clearly - they become link targets
- Use `zettelkasten`-style IDs if you prefer: `202401151234 Topic`
- Keep notes focused on one concept (atomic notes)

## 🔄 Coming from Notion: Common Translations

| Notion Concept | Obsidian Approach |
|----------------|-------------------|
| Database | Dataview plugin + frontmatter |
| Linked databases | Links + backlinks |
| Nested pages | Links (not hierarchy) |
| Templates | Templates plugin |
| Synced blocks | Note embeds |
| Relations | Links + frontmatter properties |

## 🎯 Your First Week Plan

1. **Day 1-2**: Just write notes and use `[[links]]` between them
2. **Day 3**: Try Daily Notes for journaling/tasks
3. **Day 4**: Explore Graph View to see your knowledge network
4. **Day 5**: Add frontmatter to a few notes
5. **Day 6**: Install Dataview plugin
6. **Day 7**: Create a MOC (Map of Content) note that links to related notes

## 💡 Pro Tips
- **Don't over-organize early** - let structure emerge from links
- **Use aliases** for notes you might search with different names
- **Try the Zen mode** for distraction-free writing
- **Sync** with Obsidian Sync or use Git (free version control!)

## ❓ When You Get Stuck
Remember: In Obsidian, you're just working with text files. You can always open your vault folder in another text editor if needed.

The biggest adjustment from Notion will be **thinking in links rather than hierarchies**, but once you experience how your notes become a connected web of knowledge, it's transformative.

Want me to dive deeper into any specific feature or workflow?