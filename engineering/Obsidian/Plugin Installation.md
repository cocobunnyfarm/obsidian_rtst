
# Installing Plugins in Obsidian: Complete Guide

## 📦 Two Types of Plugins

### 1. **Core Plugins** (Built-in, by Obsidian Team)
### 2. **Community Plugins** (Third-party, from developers)

---

## 🔧 **Step-by-Step Installation**

### **Step 1: Enable Community Plugins**
1. Open **Settings** (`Cmd/Ctrl + ,`)
2. Go to **Community plugins** tab
3. Click **Turn on community plugins** (first time only)
4. **Disable Safe Mode** if prompted (it's safe, I promise!)

![Settings path: Settings → Community plugins → Turn on](https://i.imgur.com/help-image.png)

### **Step 2: Browse & Install**
1. Click **Browse** in Community plugins
2. Search or browse plugins
3. Click on a plugin to see details
4. Click **Install**

### **Step 3: Enable & Configure**
1. After installing, **toggle the switch ON**
2. Some plugins need configuration:
   - Go to **Installed plugins** list
   - Click **Settings** (gear icon) next to plugin
3. Restart Obsidian if prompted

---

## 🚀 **Quick Installation Cheat Sheet**

```bash
1. Open Settings (Cmd/Ctrl + ,)
2. Click "Community Plugins"
3. Click "Browse"
4. Search → Install → Enable
5. Configure if needed
```

---

## 🔍 **Finding Good Plugins**

### **Inside Obsidian:**
- **Browse** tab: Featured, popular, new updates
- **Search**: Use keywords like "tasks", "git", "math"
- **Sort by**: Trending, updates, installs

### **External Resources:**
- **[Obsidian Plugin Stats](https://obsidian-plugin-stats.vercel.app/)**: Most popular plugins
- **[Obsidian Hub](https://publish.obsidian.md/hub)**: Plugin categories & reviews
- **Reddit r/ObsidianMD**: Recommendations

---

## 📁 **Manual Installation (Advanced)**
If a plugin isn't in the community list:
1. Download from GitHub
2. Extract to: `YourVault/.obsidian/plugins/plugin-name/`
3. Enable in settings

---

## 🛠 **Essential Plugins to Install First**

### **For Everyone:**
1. **Templater** - Advanced templates
2. **Dataview** - Query your notes like a database
3. **QuickAdd** - Quick note creation
4. **Calendar** - Daily notes integration

### **For Programmers:**
```yaml
1. Obsidian Git - Version control
2. Code Styler - Better code blocks
3. Execute Code - Run code in notes
4. GitHub Publisher - Sync to GitHub
```

### **For Mathematics:**
```yaml
1. MathBooster - Enhanced LaTeX
2. Excalidraw - Diagrams & sketches
3. Obsidian Charts - Data visualization
```

---

## ⚙️ **Managing Your Plugins**

### **Update Plugins:**
- Settings → Community plugins → **Check for updates**
- **Update all** button appears when updates available

### **Disable/Remove:**
- **Disable**: Toggle switch OFF (keeps settings)
- **Uninstall**: Click **Uninstall** (removes completely)

### **Troubleshoot Conflicts:**
1. Disable all plugins
2. Enable one by one to find conflict
3. Check plugin documentation
4. Visit plugin GitHub page for issues

---

## 🎯 **Best Practices**

### **1. Don't Install Everything**
- Start with 3-5 plugins max
- Add as you need features
- Too many plugins = slower performance

### **2. Read Documentation**
- Click **Plugin website** link in plugin info
- Check GitHub for usage examples
- Look for YouTube tutorials

### **3. Backup Your Config**
- Export plugin settings if possible
- `.obsidian` folder contains all configs
- Sync via Git or cloud storage

### **4. Plugin Safety**
- Check install count (10k+ = generally safe)
- Look at last update date (recent = maintained)
- Read issue reports on GitHub

---

## 💡 **Pro Tips**

### **Plugin Groups:**
Create themed plugin sets:
- **Writing mode**: Minimal plugins
- **Research mode**: Citations, PDF, Dataview
- **Coding mode**: Git, code execution, snippets

### **Keyboard Shortcuts:**
- `Cmd/Ctrl + P` → Search plugins
- Many plugins add their own shortcuts
- Check plugin settings for shortcut config

### **Plugin Combinations:**
```
Templater + QuickAdd = Automated workflows
Dataview + Tasks = Smart task management
Canvas + Excalidraw = Visual thinking
```

---

## 🚨 **Common Issues & Fixes**

### **"Plugin not loading":**
1. Check if it's enabled
2. Restart Obsidian
3. Reinstall plugin
4. Check console (`Ctrl+Shift+I` → Console)

### **"Slow performance":**
1. Disable heavy plugins
2. Reduce number of plugins
3. Check plugin memory usage

### **"Missing features":**
1. Check plugin settings (many are disabled by default)
2. Read the README on GitHub
3. Look for companion plugins

---

## 📝 **Quick Exercise: Install Your First Plugin**

**Let's install Dataview (super useful):**
1. Open Settings (`Cmd/Ctrl + ,`)
2. Click "Community plugins"
3. Click "Browse"
4. Search "Dataview"
5. Click "Install"
6. Toggle switch ON
7. Create a new note and try:
```dataview
LIST
FROM #todo
```

---

## 🔄 **Syncing Plugins Across Devices**

### **Using Obsidian Sync:**
- Plugins sync automatically
- Settings sync too

### **Manual Sync:**
1. Copy `.obsidian/plugins/` folder
2. Copy `.obsidian/community-plugins.json`
3. Paste to other devices

### **Using Git:**
```bash
# Add to .gitignore exceptions
!.obsidian/community-plugins.json
!.obsidian/plugins/
```

---

Need help with specific plugins or having installation issues? Tell me which plugin you're trying to install, or what functionality you're looking for!