# Robotics Development Pipeline - Desktop App

This project has been converted to support both web and desktop versions using Electron.

## Available Commands

### Web Version
```bash
npm run dev          # Run development server (web)
npm run build        # Build for production (web)
npm run start        # Start production server (web)
```

### Desktop Version
```bash
npm run electron:dev     # Run development version (desktop)
npm run electron:pack    # Package desktop app
npm run electron:dist    # Build distributable desktop app
```

## Development Workflow

1. **Web Development**: Use `npm run dev` for web development as usual
2. **Desktop Testing**: Use `npm run electron:dev` to test the desktop version
3. **Desktop Building**: Use `npm run electron:dist` to create installable Windows exe

## Desktop Features

- Native Windows application
- Custom menu bar with robotics-specific actions
- Keyboard shortcuts:
  - `Ctrl+N` - New Project
  - `Ctrl+O` - Open Project
  - `F5` - Run Simulation
  - `Ctrl+E` - Export Report
  - `Ctrl+L` - Toggle LLM Assistant
- Desktop notifications
- File system integration
- Offline capability

## Building for Distribution

To create an installer for Windows:

```bash
npm run electron:dist
```

This will create:
- `dist/Robotics Development Pipeline Setup.exe` - Windows installer
- The installer includes auto-updater support and creates desktop shortcuts

## Technical Notes

- The desktop app uses Electron to wrap the Next.js application
- Static export is used for the production build
- All Tailwind CSS and React features are preserved
- The app maintains the same UI/UX as the web version
