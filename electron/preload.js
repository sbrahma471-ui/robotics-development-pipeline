// Create a simple preload script for security
const { contextBridge, ipcRenderer } = require('electron')

contextBridge.exposeInMainWorld('electronAPI', {
  // Add any APIs you want to expose to the renderer process
  newProject: () => ipcRenderer.invoke('new-project'),
  openProject: () => ipcRenderer.invoke('open-project'),
  runSimulation: () => ipcRenderer.invoke('run-simulation'),
  exportReport: () => ipcRenderer.invoke('export-report'),
  toggleLLMPanel: () => ipcRenderer.invoke('toggle-llm-panel'),
  showAbout: () => ipcRenderer.invoke('show-about')
})
