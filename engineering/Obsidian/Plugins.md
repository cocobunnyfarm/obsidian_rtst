
# Obsidian Canvas Deep Dive

## 🎨 Canvas Basics

### Creating & Accessing
- **New canvas**: `Cmd/Ctrl+N` → select "Canvas" or create `.canvas` file
- **Open existing**: Click any `.canvas` file in your vault
- **From command palette**: "Canvas: Create new canvas"

### Core Elements

**1. Cards (Notes)**
- Drag notes from file explorer onto canvas
- Or type `[[Note Name]]` directly on canvas
- **Double-click** to edit note content inline
- **Resize** by dragging edges

**2. Text Cards**
- Click anywhere and start typing
- Full markdown support
- Can contain links to notes

**3. Media Cards**
- Drag images, PDFs, videos onto canvas
- Images can be resized and positioned

**4. Grouping & Connections**
- **Draw connections**: Click edge of card → drag to another card
- **Label connections**: Click on connection line to add text
- **Group cards**: Select multiple → right click → "Group"
- **Arrange**: Auto-layout options in top toolbar

## 🚀 Advanced Canvas Techniques

### 1. Layered Organization
```
Top Layer: Main concepts (large cards)
Middle: Supporting notes (medium)
Bottom: References & details (small, grouped)
```

### 2. Workflow Canvas Example
```yaml
Left Column: Inputs (articles, videos, ideas)
Center: Processing (notes being written)
Right Column: Outputs (projects, essays)
Connections: Show how inputs become outputs
```

### 3. Embedded Canvases
- Create a canvas and embed it in a note with `![[Canvas Name.canvas]]`
- Great for creating dashboards

### 4. Canvas as Project Board
```
[Backlog] → [In Progress] → [Review] → [Done]
Drag note cards between columns as they progress
```

## 👨‍💻 Programming & Coding Plugins

### Essential Coding Plugins

**1. Code Styler** (Community)
- Syntax highlighting customization
- Line numbers, copy buttons
- Theme matching your editor

**2. Execute Code** (Community)
```javascript
// Write and execute code blocks
const result = 5 + 3;
console.log(result);
```
- Supports: Python, JavaScript, Java, C++, etc.
- Shows output directly in note

**3. Git Integration**
- **Obsidian Git** (Community): Auto-commit, diff view, branch management
- Perfect for code documentation

**4. Developer Tools**
- **QuickAdd**: Automate note creation with templates
- **Templater**: Advanced templates with JavaScript execution
- **CustomJS**: Add custom JavaScript to your vault

### Code-Specific Workflows

**Snippet Library**
```markdown
## Python Snippets
```python
# [[Python/DataFrame Cleanup]]
def clean_df(df):
    return df.dropna()
`` `
```

**API Documentation**
```markdown
## API Endpoints
```rest
GET /api/users
Authorization: Bearer {token}
`` `

[[Implementation Details]]
[[Testing Results]]
```

**Code Review Template**
```yaml
---
review_date: 2024-01-15
pr_number: 123
status: pending
---
## Changes Made
![[code-diff.png]]

## Issues Found
- [ ] Memory leak in `process_data()`
- [ ] Missing error handling

[[Related PRs]]
```

## ➗ Mathematics & Scientific Plugins

### 1. LaTeX Support (Built-in)
```
Inline: $E = mc^2$

Display:
$$
\int_{-\infty}^{\infty} e^{-x^2} dx = \sqrt{\pi}
$$
```

### 2. MathBooster (Community)
- Equation numbering
- Theorem environments
- Cross-references to equations
```latex
$$
\begin{align}
y &= mx + b \tag{1.1} \\
a^2 + b^2 &= c^2 \tag{1.2}
\end{align}
$$
```

### 3. Excalidraw (Community)
- Hand-drawn diagrams with LaTeX support
- Perfect for mathematical diagrams
- Embeddable in notes

### 4. Obsidian Charts (Community)
```chart
type: line
labels: [Jan, Feb, Mar]
series:
  - title: Sales
    data: [120, 150, 180]
  - title: Expenses
    data: [80, 90, 85]
```

## 🧩 Specialized Plugin Recommendations

### For Researchers/Academics
- **Citations**: Zotero integration
- **Academic Notes**: Templates for papers, annotations
- **Literature Notes**: Connect papers to ideas

### For Project Management
- **Projects**: Kanban boards with tasks
- **Tasks**: Task management with queries
- **Calendar**: Time-based planning

### For Writers
- **Longform**: Novel/chapter management
- **Outliner**: Enhanced list navigation
- **Enhanced Editing**: Better writing flow

## 🔧 Installation Guide

1. Open Settings → Community Plugins
2. Disable "Safe Mode"
3. Browse or search for plugins
4. Install and Enable
5. Configure in plugin settings

## 💻 Programmer's Obsidian Setup Example

### Folder Structure
```
vault/
├── Code/
│   ├── Snippets/
│   ├── Algorithms/
│   └── API-Docs/
├── Projects/
│   ├── Project-A/
│   │   ├── Requirements.md
│   │   ├── Architecture.canvas
│   │   └── Tasks.md
│   └── Project-B/
├── Knowledge/
│   ├── Languages/
│   ├── Frameworks/
│   └── Concepts/
└── Daily/
    └── 2024-01-15.md
```

### Daily Note Template for Developers
```yaml
---
tags: daily
projects: [obsidian-setup]
---
## Today's Tasks
- [ ] Fix bug in [[Project-A]]
- [ ] Review [[PR-123]]

## Code Snippets Created
```python
# [[New Helper Function]]
def parse_response(data):
    return json.loads(data)
`` `

## Learning
Reading about [[Graph Algorithms]] for [[Project-B]]
```

## 🎯 Quick Tips

### Canvas Pro Tips:
1. **Keyboard shortcuts**: 
   - `A` → Add card
   - `E` → Embed selected note
   - `Shift-click` → Select multiple

2. **Organization**:
   - Use background colors for different card types
   - Create "zones" with labeled groups
   - Use arrows to indicate flow direction

3. **Presentation Mode**:
   - Canvas has built-in presentation mode
   - Great for code architecture reviews

### Mathematics Workflow:
1. Write equations in LaTeX
2. Reference them with `\eqref{}`
3. Create proof canvases linking definitions → theorems → corollaries

### Coding Workflow:
1. Document functions in notes
2. Link to implementation files
3. Use Dataview to generate API documentation:
```dataview
TABLE parameters, returns
FROM #function 
WHERE language = "python"
SORT file.name
```

## 🔗 Recommended Starter Plugin List

For programmers:
1. **Git** - version control
2. **Execute Code** - run snippets
3. **Code Styler** - better syntax
4. **Dataview** - query your notes
5. **Templater** - smart templates

For mathematicians:
1. **MathBooster** - advanced LaTeX
2. **Excalidraw** - diagrams
3. **Citations** - paper management
4. **Canvas** - visual proofs

Would you like me to show you specific canvas setups for programming projects or mathematical note-taking? I can provide templates for either!