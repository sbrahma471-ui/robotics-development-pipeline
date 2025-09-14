// Create a secure preload script for file operations and VS Code integration
const { contextBridge, ipcRenderer } = require('electron')
const { shell } = require('electron')
const fs = require('fs')
const path = require('path')
const { exec } = require('child_process')

contextBridge.exposeInMainWorld('electronAPI', {
  // Existing APIs
  newProject: () => ipcRenderer.invoke('new-project'),
  openProject: () => ipcRenderer.invoke('open-project'),
  runSimulation: () => ipcRenderer.invoke('run-simulation'),
  exportReport: () => ipcRenderer.invoke('export-report'),
  toggleLLMPanel: () => ipcRenderer.invoke('toggle-llm-panel'),
  showAbout: () => ipcRenderer.invoke('show-about'),
  
  // File system operations
  writeFile: (filePath, content) => {
    try {
      // Ensure directory exists
      const dir = path.dirname(filePath)
      if (!fs.existsSync(dir)) {
        fs.mkdirSync(dir, { recursive: true })
      }
      fs.writeFileSync(filePath, content, 'utf8')
      return { success: true }
    } catch (error) {
      return { success: false, error: error.message }
    }
  },
  
  readFile: (filePath) => {
    try {
      if (fs.existsSync(filePath)) {
        const content = fs.readFileSync(filePath, 'utf8')
        return { success: true, content }
      } else {
        return { success: false, error: 'File not found' }
      }
    } catch (error) {
      return { success: false, error: error.message }
    }
  },
  
  fileExists: (filePath) => {
    return fs.existsSync(filePath)
  },
  
  // VS Code integration
  openInVSCode: (filePath) => {
    return new Promise((resolve) => {
      // Try multiple methods to open VS Code
      const commands = [
        `code "${filePath}"`,
        `"C:\\Program Files\\Microsoft VS Code\\Code.exe" "${filePath}"`,
        `"C:\\Users\\${process.env.USERNAME}\\AppData\\Local\\Programs\\Microsoft VS Code\\Code.exe" "${filePath}"`
      ]
      
      let commandIndex = 0
      
      const tryCommand = () => {
        if (commandIndex >= commands.length) {
          // If all commands fail, try shell.openExternal as fallback
          shell.openExternal(`vscode://file/${filePath}`)
            .then(() => resolve({ success: true, method: 'shell.openExternal' }))
            .catch(() => resolve({ success: false, error: 'All VS Code opening methods failed' }))
          return
        }
        
        exec(commands[commandIndex], (error) => {
          if (error) {
            commandIndex++
            tryCommand()
          } else {
            resolve({ success: true, method: `command: ${commands[commandIndex]}` })
          }
        })
      }
      
      tryCommand()
    })
  },
  
  // Get app paths
  getAppPath: () => {
    return process.cwd()
  },
  
  // Platform info
  getPlatform: () => {
    return {
      platform: process.platform,
      isElectron: true,
      nodeVersion: process.version
    }
  }
})
