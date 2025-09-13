const { app, BrowserWindow, Menu } = require('electron')
const path = require('path')
const fs = require('fs')

// Better development detection
const isDev = !app.isPackaged || process.env.NODE_ENV !== 'production'

console.log('Development mode:', isDev)
console.log('NODE_ENV:', process.env.NODE_ENV)
console.log('app.isPackaged:', app.isPackaged)

function createWindow() {
  const mainWindow = new BrowserWindow({
    width: 1400,
    height: 900,
    minWidth: 1200,
    minHeight: 800,
    webPreferences: {
      nodeIntegration: false,
      contextIsolation: true,
      enableRemoteModule: false,
      webSecurity: true
    },
    icon: path.join(__dirname, '../public/favicon.ico'),
    titleBarStyle: 'default',
    show: false
  })

  // Load the app - Force development mode when running from npm script
  const isDevMode = !app.isPackaged || process.argv.includes('--dev')
  
  console.log('Loading in development mode:', isDevMode)
  
  if (isDevMode) {
    // Try to connect to development server
    const loadDevServer = () => {
      mainWindow.loadURL('http://localhost:3001')
        .then(() => {
          console.log('Successfully connected to localhost:3001')
        })
        .catch(() => {
          console.log('Failed to connect to 3001, trying 3000...')
          return mainWindow.loadURL('http://localhost:3000')
        })
        .then(() => {
          console.log('Successfully connected to localhost:3000')
        })
        .catch(() => {
          console.log('Failed to connect to development server')
          mainWindow.loadURL('data:text/html,<div style="padding:50px;font-family:Arial;text-align:center"><h1>🤖 Robotics Development Pipeline</h1><h2>Development Server Not Found</h2><p>Please make sure Next.js is running with <code style="background:#f0f0f0;padding:2px 6px;border-radius:3px">npm run dev</code></p><p>Then restart the Electron app.</p><br><button onclick="location.reload()" style="padding:10px 20px;background:#007acc;color:white;border:none;border-radius:5px;cursor:pointer">Retry Connection</button></div>')
        })
    }
    
    // Load after a short delay to ensure Next.js is ready
    setTimeout(loadDevServer, 2000)
    
    // Open DevTools in development
    mainWindow.webContents.openDevTools()
  } else {
    mainWindow.loadFile(path.join(__dirname, '../out/index.html'))
  }

  // Show window when ready to prevent visual flash
  mainWindow.once('ready-to-show', () => {
    mainWindow.show()
  })

  // Create application menu
  const template = [
    {
      label: 'File',
      submenu: [
        {
          label: 'New Project',
          accelerator: 'CmdOrCtrl+N',
          click: () => {
            // Add new project functionality
            mainWindow.webContents.send('new-project')
          }
        },
        {
          label: 'Open Project',
          accelerator: 'CmdOrCtrl+O',
          click: () => {
            // Add open project functionality
            mainWindow.webContents.send('open-project')
          }
        },
        { type: 'separator' },
        {
          label: 'Exit',
          accelerator: process.platform === 'darwin' ? 'Cmd+Q' : 'Ctrl+Q',
          click: () => {
            app.quit()
          }
        }
      ]
    },
    {
      label: 'View',
      submenu: [
        { role: 'reload' },
        { role: 'forceReload' },
        { role: 'toggleDevTools' },
        { type: 'separator' },
        { role: 'resetZoom' },
        { role: 'zoomIn' },
        { role: 'zoomOut' },
        { type: 'separator' },
        { role: 'togglefullscreen' }
      ]
    },
    {
      label: 'Robotics',
      submenu: [
        {
          label: 'Run Simulation',
          accelerator: 'F5',
          click: () => {
            mainWindow.webContents.send('run-simulation')
          }
        },
        {
          label: 'Export Report',
          accelerator: 'CmdOrCtrl+E',
          click: () => {
            mainWindow.webContents.send('export-report')
          }
        },
        { type: 'separator' },
        {
          label: 'Open LLM Assistant',
          accelerator: 'CmdOrCtrl+L',
          click: () => {
            mainWindow.webContents.send('toggle-llm-panel')
          }
        }
      ]
    },
    {
      label: 'Help',
      submenu: [
        {
          label: 'About Robotics Pipeline',
          click: () => {
            // Add about dialog
            mainWindow.webContents.send('show-about')
          }
        }
      ]
    }
  ]

  const menu = Menu.buildFromTemplate(template)
  Menu.setApplicationMenu(menu)
}

// This method will be called when Electron has finished initialization
app.whenReady().then(createWindow)

// Quit when all windows are closed
app.on('window-all-closed', () => {
  if (process.platform !== 'darwin') {
    app.quit()
  }
})

app.on('activate', () => {
  if (BrowserWindow.getAllWindows().length === 0) {
    createWindow()
  }
})

// Security: Prevent new window creation
app.on('web-contents-created', (event, contents) => {
  contents.on('new-window', (event, navigationUrl) => {
    event.preventDefault()
  })
})
