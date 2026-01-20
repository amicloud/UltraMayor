# SealSlicer Todo

## 💻 UI
- Determine how the various modules should appear in the UI
    - Taking inspiration from Blender and other slicers
### 📃 Data 
- Material viewer/creator/editor
- Printer creator/editor
- Slice profiles creator/editor
- User profile/settings viewer/editor
### ⌨️ Input
- Add keyboard shortcuts
    - Undo & Redo
    - Camera WASD
    - Camera Numpad Controls like Blender
- Click detection in renderer
    - Drag objects
### 🎨 Themes
- Dark, Light
- Detect system theme
- Create custom themes, store as toml
### 👜 Misc
- Object groups
- Build plate texture
---
## 🎞️ Mesh Renderer
- Optimize rendering pipeline
---
## 🔪 MSLA Slicer
- Figure out a way to test the sliced images
- Implement way more tests
### Mesh Island Analyzer
- Write more tests and maybe try to optimize
--- 
## 🥚 Project Files
### 🥚 Egg Files
- Create a project file type .egg
    - Provide the option of embedding the mesh data in the egg or referencing it as a separate ~~fish~~ file
        - This could allow for really neat and data efficient things
--- 
## 📤 Data Export
### 📁 File types to do
- STL
- OBJ
- 3MF
- CTB
---
## 📥 Data Import
### 📁 File types to do
- ~~STL~~
- OBJ
- 3MF
- CTB?
---
## 📬 Distribution
- Figure out appimage
    - And automatic building of them