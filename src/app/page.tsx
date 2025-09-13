"use client"

import React, { useState } from "react"
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card"
import { Badge } from "@/components/ui/badge"
import { Button } from "@/components/ui/button"
import { Progress } from "@/components/ui/progress"
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs"
import { Dialog, DialogContent, DialogDescription, DialogHeader, DialogTitle } from "@/components/ui/dialog"
import { Textarea } from "@/components/ui/textarea"
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "@/components/ui/select"
import { ScrollArea } from "@/components/ui/scroll-area"
import {
  Box,
  Cog,
  Play,
  HardDrive,
  FileText,
  Database,
  Brain,
  GitBranch,
  ArrowRight,
  CheckCircle,
  Clock,
  AlertCircle,
  Settings,
  Zap,
  Monitor,
  Download,
  X,
  Send,
  Upload,
  Code,
  Eye,
  CloudLightningIcon as Lightning,
  Copy,
  RefreshCw,
  ImageIcon,
  ChevronRight,
  Split,
  FileBarChart,
} from "lucide-react"

interface WorkflowStep {
  id: string
  title: string
  description: string
  icon: React.ReactNode
  status: "completed" | "in-progress" | "pending"
  progress: number
  tools: string[]
  integrations?: string[]
  branches?: string[]
}

interface SimulationEnvironment {
  name: string
  status: "active" | "idle" | "error"
  progress: number
  instances: number
}

interface TestScenario {
  name: string
  status: "completed" | "running" | "queued" | "failed"
  duration: string
  success: boolean
  testType: string
  environment: string
  details: string
}

interface ChatMessage {
  id: string
  type: "user" | "assistant"
  content: string
  timestamp: string
  codeSnippet?: string
  imageUrl?: string
  relatedStep?: string
}

interface VLAModel {
  id: string
  name: string
  description: string
  capabilities: string[]
}

interface CertificationStage {
  stage: string
  status: "completed" | "in-progress" | "pending"
  date: string
  description: string
  vendor: string
  details?: string
}

interface PerformanceMetric {
  name: string
  current: number
  target: number
  unit: string
  status: "excellent" | "good" | "warning" | "critical"
}

interface Observation {
  category: string
  observation: string
  recommendation: string
  priority: "High" | "Medium" | "Low"
  impact: string
}

// Enhanced test scenarios with more detailed information
const testScenarios: TestScenario[] = [
  {
    name: "Navigation Test",
    status: "completed",
    duration: "2m 34s",
    success: true,
    testType: "Automated Navigation",
    environment: "Gazebo Simulation",
    details:
      "Robot successfully navigated through complex obstacle course with 98% accuracy. Tested dynamic obstacle avoidance, path replanning, and emergency stops.",
  },
  {
    name: "Manipulation Task",
    status: "running",
    duration: "1m 12s",
    success: false,
    testType: "Object Manipulation",
    environment: "Genesis Simulation",
    details:
      "Currently testing pick-and-place operations with various object geometries. Testing grasping strategies for objects weighing 0.5kg to 4.8kg.",
  },
  {
    name: "Collision Avoidance",
    status: "completed",
    duration: "3m 45s",
    success: true,
    testType: "Safety Testing",
    environment: "Gazebo Simulation",
    details:
      "Collision detection system responded within 50ms threshold in all test cases. Tested with static and dynamic obstacles at various speeds.",
  },
  {
    name: "Path Planning",
    status: "queued",
    duration: "0s",
    success: false,
    testType: "Algorithm Testing",
    environment: "IsaacSim",
    details:
      "Waiting for IsaacSim environment to become available. Will test RRT*, A*, and custom path planning algorithms.",
  },
  {
    name: "Sensor Calibration",
    status: "failed",
    duration: "45s",
    success: false,
    testType: "Hardware Integration",
    environment: "Physical Hardware",
    details:
      "Sensor calibration failed due to hardware communication timeout. Camera and LIDAR sensors need recalibration.",
  },
  {
    name: "Multi-Robot Coordination",
    status: "completed",
    duration: "4m 18s",
    success: true,
    testType: "Swarm Robotics",
    environment: "Gazebo Multi-Robot",
    details:
      "Successfully coordinated 3 robotic arms in collaborative assembly task. Communication latency averaged 12ms.",
  },
  {
    name: "Real-time Performance",
    status: "completed",
    duration: "5m 02s",
    success: true,
    testType: "Performance Testing",
    environment: "Hardware + Simulation",
    details:
      "Maintained 1000Hz control loop frequency with 99.8% consistency. CPU usage peaked at 72% during complex maneuvers.",
  },
]

// Enhanced observations with more detailed analysis
const observations: Observation[] = [
  {
    category: "Performance",
    observation: "Robotic arm consistently achieved target positions with high accuracy across all test scenarios",
    recommendation:
      "Maintain current PID controller parameters for production deployment. Consider fine-tuning for specific payload ranges.",
    priority: "Low",
    impact: "System performance exceeds requirements by 1.8%. Maintaining current settings ensures reliability.",
  },
  {
    category: "Safety",
    observation:
      "Collision detection system responded within acceptable thresholds but shows minor delays with fast-moving obstacles",
    recommendation:
      "Consider reducing safety margins by 5% to improve task completion time while upgrading sensor refresh rate to 100Hz",
    priority: "Medium",
    impact: "Could improve overall task completion time by 8-12% while maintaining safety standards",
  },
  {
    category: "Efficiency",
    observation: "Path planning algorithm showed suboptimal routes in complex scenarios with multiple obstacles",
    recommendation: "Implement RRT* algorithm for improved path optimization and add dynamic replanning capabilities",
    priority: "High",
    impact: "Expected 15-20% improvement in path efficiency and 25% reduction in task completion time",
  },
  {
    category: "Hardware",
    observation: "Sensor calibration failures occurred in 12% of hardware integration tests",
    recommendation: "Implement automated sensor health monitoring and self-calibration routines",
    priority: "High",
    impact: "Would reduce manual intervention by 80% and improve system reliability",
  },
  {
    category: "Communication",
    observation: "Multi-robot coordination showed excellent performance with low latency communication",
    recommendation: "Scale testing to 5+ robots to validate communication protocol under higher loads",
    priority: "Medium",
    impact: "Validates scalability for industrial deployment scenarios",
  },
]

// Enhanced certification pipeline with more detailed stages
const certificationPipeline: CertificationStage[] = [
  {
    stage: "Documentation Review",
    status: "completed",
    date: "2024-01-10",
    description: "Initial documentation and technical specification review",
    vendor: "RoboTech Certification Ltd.",
    details:
      "All technical documentation, safety protocols, and design specifications reviewed and approved. Minor formatting corrections requested and completed.",
  },
  {
    stage: "Safety Assessment",
    status: "completed",
    date: "2024-01-12",
    description: "Comprehensive safety analysis and risk assessment",
    vendor: "RoboTech Certification Ltd.",
    details:
      "Safety systems evaluated including emergency stops, collision detection, and fail-safe mechanisms. All safety requirements met or exceeded.",
  },
  {
    stage: "Report Submission",
    status: "completed",
    date: "2024-01-15",
    description: "Test report shared with certification vendor",
    vendor: "RoboTech Certification Ltd.",
    details:
      "Complete technical documentation and test results submitted for review. Includes performance metrics, safety analysis, and compliance verification.",
  },
  {
    stage: "Vendor Review",
    status: "completed",
    date: "2024-01-18",
    description: "Technical review and validation by vendor engineers",
    vendor: "RoboTech Certification Ltd.",
    details:
      "All technical specifications and safety requirements validated successfully. Independent testing confirmed reported performance metrics.",
  },
  {
    stage: "Test Approval",
    status: "completed",
    date: "2024-01-20",
    description: "Formal approval of test results and methodology",
    vendor: "RoboTech Certification Ltd.",
    details:
      "Test methodology and results approved with minor recommendations for documentation. Performance exceeds industry standards.",
  },
  {
    stage: "Compliance Verification",
    status: "completed",
    date: "2024-01-21",
    description: "Verification of regulatory compliance and standards adherence",
    vendor: "RoboTech Certification Ltd.",
    details:
      "Confirmed compliance with ISO 10218-1, ISO 10218-2, and relevant safety standards. All regulatory requirements satisfied.",
  },
  {
    stage: "Certificate Initiation",
    status: "in-progress",
    date: "2024-01-22",
    description: "Certificate generation and quality assurance process",
    vendor: "RoboTech Certification Ltd.",
    details:
      "Certificate template created and populated with test results. Currently undergoing final quality assurance review and legal verification.",
  },
  {
    stage: "Final Review",
    status: "pending",
    date: "2024-01-24",
    description: "Final technical and legal review before certificate issuance",
    vendor: "RoboTech Certification Ltd.",
    details:
      "Awaiting final sign-off from senior certification engineers and legal team. Expected completion within 24 hours.",
  },
  {
    stage: "Certificate Disbursement",
    status: "pending",
    date: "2024-01-25",
    description: "Final certificate delivery and registration",
    vendor: "RoboTech Certification Ltd.",
    details:
      "Certificate will be digitally signed and delivered via secure portal. Physical certificate will be mailed within 5 business days.",
  },
]

// Add historical performance data for trends
const historicalPerformanceData = [
  {
    date: "2024-01-01",
    version: "v2.1 (Current)",
    overallSuccess: 94.2,
    taskCompletion: 127,
    averageTime: "2.34s",
    accuracy: 96.8,
    collisionRate: 0.8,
    pathEfficiency: 89.3,
  },
  {
    date: "2023-12-01",
    version: "v2.0",
    overallSuccess: 91.5,
    taskCompletion: 115,
    averageTime: "2.67s",
    accuracy: 94.2,
    collisionRate: 1.2,
    pathEfficiency: 85.7,
  },
  {
    date: "2023-11-01",
    version: "v1.9",
    overallSuccess: 88.3,
    taskCompletion: 98,
    averageTime: "3.12s",
    accuracy: 91.8,
    collisionRate: 1.8,
    pathEfficiency: 82.1,
  },
  {
    date: "2023-10-01",
    version: "v1.8",
    overallSuccess: 85.1,
    taskCompletion: 87,
    averageTime: "3.45s",
    accuracy: 89.5,
    collisionRate: 2.3,
    pathEfficiency: 78.9,
  },
  {
    date: "2023-09-01",
    version: "v1.7",
    overallSuccess: 82.7,
    taskCompletion: 76,
    averageTime: "3.78s",
    accuracy: 87.2,
    collisionRate: 2.9,
    pathEfficiency: 75.4,
  },
]

// Add benchmark targets for comparison
const benchmarkTargets = {
  overallSuccess: 95.0,
  taskCompletion: 150,
  averageTime: "2.00s",
  accuracy: 98.0,
  collisionRate: 0.5,
  pathEfficiency: 92.0,
}

const vlaModels: VLAModel[] = [
  {
    id: "gpt-4-vision",
    name: "GPT-4 Vision",
    description: "Advanced vision-language model for code analysis",
    capabilities: ["Vision", "Language", "Code Generation"],
  },
  {
    id: "claude-3-vision",
    name: "Claude 3 Vision",
    description: "Anthropic's vision-language model",
    capabilities: ["Vision", "Language", "Safety Analysis"],
  },
  {
    id: "gemini-pro-vision",
    name: "Gemini Pro Vision",
    description: "Google's multimodal AI for robotics",
    capabilities: ["Vision", "Language", "Action Planning"],
  },
  {
    id: "rt-2",
    name: "RT-2 (Robotics Transformer)",
    description: "Specialized VLA model for robotics tasks",
    capabilities: ["Vision", "Language", "Action", "Robotics"],
  },
]

const quickPrompts = [
  "Analyze my simulation performance and suggest optimizations",
  "Generate URDF code for a robotic arm",
  "Debug collision detection issues in Gazebo",
  "Optimize path planning algorithm",
  "Generate ROS launch file for multi-robot simulation",
  "Analyze sensor data and suggest calibration",
]

// CORRECTED workflow order to match the PDF diagram with branching
const workflowSteps: WorkflowStep[] = [
  {
    id: "cad",
    title: "CAD Design",
    description: "3D modeling and mechanical design using Fusion360",
    icon: <Box className="h-6 w-6" />,
    status: "completed",
    progress: 100,
    tools: ["Fusion360", "SolidWorks", "AutoCAD"],
    integrations: ["GitHub", "Version Control"],
  },
  {
    id: "mech-design",
    title: "Mechanical Design",
    description: "Convert CAD models to URDF/XML format for simulation",
    icon: <Cog className="h-6 w-6" />,
    status: "completed",
    progress: 85,
    tools: ["URDF", "XML", "ROS"],
    integrations: ["GitHub", "Bitbucket"],
  },
  {
    id: "control",
    title: "Control System",
    description: "Design control algorithms and system architecture",
    icon: <Settings className="h-6 w-6" />,
    status: "in-progress",
    progress: 70,
    tools: ["ROS Control", "PID Controllers", "State Machines"],
    integrations: ["GitHub", "JIRA"],
    branches: ["hardware", "simulation"],
  },
  {
    id: "hardware",
    title: "Hardware Deployment",
    description: "Deploy control systems to physical hardware",
    icon: <HardDrive className="h-6 w-6" />,
    status: "pending",
    progress: 0,
    tools: ["ROS", "Hardware Controllers", "Sensors"],
    integrations: ["GitHub", "CI/CD"],
  },
  {
    id: "data",
    title: "Data Collection",
    description: "Gather data from hardware experiments and training",
    icon: <Database className="h-6 w-6" />,
    status: "pending",
    progress: 0,
    tools: ["Data Logger", "Sensors", "Analytics"],
    integrations: ["Database", "Cloud Storage"],
  },
  {
    id: "simulation",
    title: "Simulation",
    description: "Test and validate designs in virtual environments",
    icon: <Play className="h-6 w-6" />,
    status: "in-progress",
    progress: 60,
    tools: ["Gazebo", "Genesis", "IsaacSim"],
    integrations: ["JIRA", "GitHub"],
  },
  {
    id: "export",
    title: "Export & Reports",
    description: "Generate documentation and export project files",
    icon: <FileText className="h-6 w-6" />,
    status: "pending",
    progress: 0,
    tools: ["PDF Generator", "Documentation", "File Export"],
    integrations: ["JIRA", "Confluence"],
  },
]

const simulationEnvironments: SimulationEnvironment[] = [
  { name: "Gazebo", status: "active", progress: 75, instances: 3 },
  { name: "Genesis", status: "active", progress: 45, instances: 2 },
  { name: "IsaacSim", status: "idle", progress: 0, instances: 0 },
]

const systemLogs = [
  { time: "14:32:15", level: "INFO", message: "Gazebo simulation started successfully" },
  { time: "14:31:42", level: "WARN", message: "Genesis environment memory usage high (85%)" },
  { time: "14:30:18", level: "INFO", message: "Navigation test completed with 98% accuracy" },
  { time: "14:29:55", level: "ERROR", message: "IsaacSim connection timeout - retrying..." },
  { time: "14:28:33", level: "INFO", message: "URDF model loaded successfully" },
]

const integrationTools = [
  { name: "GitHub", icon: <GitBranch className="h-4 w-4" />, status: "connected" },
  { name: "Bitbucket", icon: <GitBranch className="h-4 w-4" />, status: "connected" },
  { name: "JIRA", icon: <Settings className="h-4 w-4" />, status: "connected" },
  { name: "CI/CD", icon: <Zap className="h-4 w-4" />, status: "pending" },
]

const reportSummary = {
  platform: "ROS Noetic + Gazebo 11",
  robotModel: "6-DOF Robotic Arm v2.1",
  simulation: "Multi-Environment Testing Suite",
  testObjective: "Validate robotic arm performance across manipulation tasks with 95% accuracy threshold",
  generatedDate: new Date().toLocaleDateString(),
  reportId: "RPT-2024-001",
  status: "Completed",
}

const simulationSetup = {
  softwareCompatibility: [
    { component: "ROS Noetic", status: "Compatible", version: "1.15.14" },
    { component: "Gazebo", status: "Compatible", version: "11.14.0" },
    { component: "MoveIt", status: "Compatible", version: "1.1.8" },
    { component: "OpenCV", status: "Compatible", version: "4.5.3" },
    { component: "PCL", status: "Warning", version: "1.10.0" },
  ],
  environment: {
    os: "Ubuntu 20.04 LTS",
    kernel: "5.15.0-91-generic",
    memory: "32 GB DDR4",
    gpu: "NVIDIA RTX 4090",
    storage: "2TB NVMe SSD",
  },
  robotSpecs: {
    dof: 6,
    payload: "5 kg",
    reach: "850 mm",
    repeatability: "±0.03 mm",
    maxSpeed: "250°/s",
    controllerFreq: "1000 Hz",
  },
}

const performanceMetrics: PerformanceMetric[] = [
  { name: "Overall Success Rate", current: 94.2, target: 90, unit: "%", status: "excellent" },
  { name: "Task Completion", current: 127, target: 100, unit: "tasks", status: "excellent" },
  { name: "Average Time", current: 2.34, target: 3.0, unit: "s", status: "good" },
  { name: "Accuracy", current: 96.8, target: 95, unit: "%", status: "excellent" },
  { name: "Collision Rate", current: 0.8, target: 2.0, unit: "%", status: "excellent" },
  { name: "Path Efficiency", current: 89.3, target: 85, unit: "%", status: "good" },
]

export default function RoboticsWorkflow() {
  const [selectedStep, setSelectedStep] = useState<string>("cad")
  const [activeTab, setActiveTab] = useState("workflow")
  const [showSimulationDetails, setShowSimulationDetails] = useState(false)
  const [showLLMPanel, setShowLLMPanel] = useState(false)
  const [showReportsModal, setShowReportsModal] = useState(false)
  const [showCardDetails, setShowCardDetails] = useState(false)
  const [selectedCardDetails, setSelectedCardDetails] = useState<WorkflowStep | null>(null)
  const [showExportSection, setShowExportSection] = useState(false)

  const [selectedVLAModel, setSelectedVLAModel] = useState<string>("gpt-4-vision")
  const [chatMessages, setChatMessages] = useState<ChatMessage[]>([
    {
      id: "1",
      type: "assistant",
      content:
        "Hello! I'm your VLA assistant. I can help you with any step in your robotics pipeline. Click on any workflow card to get specific assistance, or ask me anything!",
      timestamp: "14:30:00",
    },
  ])
  const [currentMessage, setCurrentMessage] = useState("")
  const [isLoading, setIsLoading] = useState(false)
  const [uploadedImage, setUploadedImage] = useState<string | null>(null)

  const getStatusIcon = (status: string) => {
    switch (status) {
      case "completed":
        return <CheckCircle className="h-4 w-4 text-green-500" />
      case "in-progress":
        return <Clock className="h-4 w-4 text-blue-500" />
      default:
        return <AlertCircle className="h-4 w-4 text-gray-400" />
    }
  }

  const getStatusColor = (status: string) => {
    switch (status) {
      case "completed":
        return "bg-green-100 text-green-800 border-green-200"
      case "in-progress":
        return "bg-blue-100 text-blue-800 border-blue-200"
      default:
        return "bg-gray-100 text-gray-600 border-gray-200"
    }
  }

  const getEnvironmentStatusColor = (status: string) => {
    switch (status) {
      case "active":
        return "bg-green-100 text-green-800"
      case "idle":
        return "bg-gray-100 text-gray-600"
      case "error":
        return "bg-red-100 text-red-800"
      default:
        return "bg-gray-100 text-gray-600"
    }
  }

  const getScenarioStatusColor = (status: string) => {
    switch (status) {
      case "completed":
        return "bg-green-100 text-green-800"
      case "running":
        return "bg-blue-100 text-blue-800"
      case "queued":
        return "bg-yellow-100 text-yellow-800"
      case "failed":
        return "bg-red-100 text-red-800"
      default:
        return "bg-gray-100 text-gray-600"
    }
  }

  const getLogLevelColor = (level: string) => {
    switch (level) {
      case "INFO":
        return "text-blue-600"
      case "WARN":
        return "text-yellow-600"
      case "ERROR":
        return "text-red-600"
      default:
        return "text-gray-600"
    }
  }

  const getMetricStatusColor = (status: string) => {
    switch (status) {
      case "excellent":
        return "text-green-600"
      case "good":
        return "text-blue-600"
      case "warning":
        return "text-yellow-600"
      case "critical":
        return "text-red-600"
      default:
        return "text-gray-600"
    }
  }

  const handleCardClick = (stepId: string) => {
    const step = workflowSteps.find((s) => s.id === stepId)
    if (step) {
      setSelectedCardDetails(step)
      setShowCardDetails(true)
      setSelectedStep(stepId)
    }
  }

  // MODIFIED: Enhanced double-click handler - Export redirects to Export Section
  const handleCardDoubleClick = (stepId: string, event?: React.MouseEvent) => {
    console.log("Double-click detected for step:", stepId) // Debug log

    // Prevent event bubbling
    if (event) {
      event.preventDefault()
      event.stopPropagation()
    }

    if (stepId === "simulation") {
      console.log("Opening simulation details modal") // Debug log
      setShowSimulationDetails(true)
    } else if (stepId === "export") {
      console.log("Opening Export & Reports section") // Debug log
      setShowExportSection(true)
    }
  }

  const handleSendMessage = async () => {
    if (!currentMessage.trim() && !uploadedImage) return

    const userMessage: ChatMessage = {
      id: Date.now().toString(),
      type: "user",
      content: currentMessage,
      timestamp: new Date().toLocaleTimeString(),
      imageUrl: uploadedImage || undefined,
      relatedStep: selectedStep,
    }

    setChatMessages((prev) => [...prev, userMessage])
    setCurrentMessage("")
    setIsLoading(true)

    // Simulate AI response
    setTimeout(() => {
      const assistantMessage: ChatMessage = {
        id: (Date.now() + 1).toString(),
        type: "assistant",
        content: generateAIResponse(currentMessage, uploadedImage, selectedStep),
        timestamp: new Date().toLocaleTimeString(),
        codeSnippet:
          currentMessage.toLowerCase().includes("code") || currentMessage.toLowerCase().includes("generate")
            ? generateCodeSnippet(currentMessage, selectedStep)
            : undefined,
        relatedStep: selectedStep,
      }
      setChatMessages((prev) => [...prev, assistantMessage])
      setIsLoading(false)
    }, 2000)

    setUploadedImage(null)
  }

  const generateAIResponse = (message: string, hasImage: boolean, stepId: string) => {
    const step = workflowSteps.find((s) => s.id === stepId)
    const stepName = step?.title || "current step"

    if (hasImage) {
      return `I can see the ${stepName.toLowerCase()} content in your image. Based on the visual analysis, I notice potential areas for improvement. Let me provide specific recommendations for your ${stepName.toLowerCase()} workflow.`
    }

    if (message.toLowerCase().includes("performance")) {
      return `For ${stepName}, I recommend optimizing performance by: 1) Streamlining your ${step?.tools.join(", ")} workflow, 2) Implementing best practices for ${stepName.toLowerCase()}, 3) Integrating with your ${step?.integrations?.join(" and ")} systems more efficiently.`
    }

    if (message.toLowerCase().includes("code") || message.toLowerCase().includes("generate")) {
      return `I'll generate optimized code for your ${stepName} workflow. This will include best practices and integration with your current tools: ${step?.tools.join(", ")}.`
    }

    return `I understand your query about ${stepName}. Based on your current progress (${step?.progress}%) and status (${step?.status}), here are my recommendations for improving your ${stepName.toLowerCase()} workflow.`
  }

  const generateCodeSnippet = (message: string, stepId: string) => {
    const step = workflowSteps.find((s) => s.id === stepId)

    if (stepId === "cad") {
      return `# CAD Design Automation Script
import fusion360_api as f360

def create_robotic_arm_base():
    # Create new design
    design = f360.create_design("RoboticArm_v1")
    
    # Create base cylinder
    base = design.create_cylinder(
        radius=50,  # mm
        height=20,  # mm
        material="Aluminum"
    )
    
    return design`
    }

    return `# ${step?.title} optimization code
def optimize_${stepId.replace("-", "_")}():
    # Implement optimization for ${step?.title}
    print("Optimizing ${step?.title}...")
    return True`
  }

  const handleQuickPrompt = (prompt: string) => {
    setCurrentMessage(prompt)
  }

  const handleImageUpload = (event: React.ChangeEvent<HTMLInputElement>) => {
    const file = event.target.files?.[0]
    if (file) {
      const reader = new FileReader()
      reader.onload = (e) => {
        setUploadedImage(e.target?.result as string)
      }
      reader.readAsDataURL(file)
    }
  }

  const copyToClipboard = (text: string) => {
    navigator.clipboard.writeText(text)
  }

  const downloadComprehensiveReport = () => {
    const reportData = {
      summary: reportSummary,
      setup: simulationSetup,
      testScenarios: testScenarios,
      performanceMetrics: performanceMetrics,
      observations: observations,
      certificationPipeline: certificationPipeline,
      workflowSteps: workflowSteps,
      generatedAt: new Date().toISOString(),
    }

    const element = document.createElement("a")
    const file = new Blob([JSON.stringify(reportData, null, 2)], { type: "application/json" })
    element.href = URL.createObjectURL(file)
    element.download = `comprehensive-robotics-report-${reportSummary.reportId}.json`
    document.body.appendChild(element)
    element.click()
    document.body.removeChild(element)
  }

  return (
    <div className="min-h-screen bg-gradient-to-br from-slate-50 to-slate-100">
      <div className="flex">
        {/* Main Content */}
        <div className={`flex-1 p-6 transition-all duration-300 ${showLLMPanel ? "mr-96" : ""}`}>
          <div className="max-w-7xl mx-auto">
            <div className="mb-8">
              <h1 className="text-4xl font-bold text-gray-900 mb-2">Robotics Development Pipeline</h1>
              <p className="text-lg text-gray-600">Complete workflow with branching control system architecture</p>
            </div>

            <Tabs value={activeTab} onValueChange={setActiveTab} className="space-y-6">
              <TabsList className="grid w-full grid-cols-3">
                <TabsTrigger value="workflow">Workflow Overview</TabsTrigger>
                <TabsTrigger value="integrations">Integrations</TabsTrigger>
                <TabsTrigger value="monitoring">Monitoring</TabsTrigger>
              </TabsList>

              <TabsContent value="workflow" className="space-y-6">
                {/* Linear Flow Steps (CAD → Mechanical → Control) */}
                <div className="flex items-center justify-center space-x-4">
                  {workflowSteps.slice(0, 3).map((step, index) => (
                    <React.Fragment key={step.id}>
                      <Card
                        className={`cursor-pointer transition-all duration-200 hover:shadow-lg w-48 ${
                          selectedStep === step.id ? "ring-2 ring-blue-500 shadow-lg" : ""
                        } ${step.id === "control" ? "ring-2 ring-orange-300" : ""}`}
                        onClick={() => handleCardClick(step.id)}
                        title="Click to view details"
                      >
                        <CardContent className="p-6 text-center">
                          <div className="flex flex-col items-center space-y-3">
                            {step.icon}
                            <CardTitle className="text-lg">{step.title}</CardTitle>
                            <div className="flex items-center space-x-2">
                              {getStatusIcon(step.status)}
                              <Badge className={getStatusColor(step.status)}>
                                {step.status.replace("-", " ").toUpperCase()}
                              </Badge>
                            </div>
                            <div className="w-full">
                              <div className="flex justify-between text-sm mb-1">
                                <span>Progress:</span>
                                <span>{step.progress}%</span>
                              </div>
                              <Progress value={step.progress} className="h-2" />
                            </div>
                            {step.id === "control" && (
                              <div className="text-xs text-orange-600 font-medium">
                                Branches to Hardware & Simulation
                              </div>
                            )}
                          </div>
                        </CardContent>
                      </Card>

                      {/* Arrow between cards */}
                      {index < 2 && (
                        <div className="flex items-center">
                          <div className="w-8 h-0.5 bg-gradient-to-r from-blue-400 to-purple-500"></div>
                          <ArrowRight className="h-6 w-6 text-purple-500 -ml-1" />
                        </div>
                      )}
                    </React.Fragment>
                  ))}
                </div>

                {/* Branching Section with Connecting Nodes */}
                <div className="relative">
                  {/* Control System to Branch Connection */}
                  <div className="flex justify-center mb-6">
                    <div className="flex flex-col items-center space-y-4">
                      {/* Connection Node from Control System */}
                      <div className="w-4 h-4 bg-orange-500 rounded-full border-2 border-orange-600"></div>
                      <div className="w-0.5 h-8 bg-orange-400"></div>

                      {/* Branch Split Node */}
                      <div className="relative">
                        <div className="w-6 h-6 bg-orange-500 rounded-full border-2 border-orange-600 flex items-center justify-center">
                          <Split className="h-3 w-3 text-white" />
                        </div>

                        {/* Horizontal connecting lines to both paths */}
                        <div className="absolute top-1/2 -left-32 w-32 h-0.5 bg-orange-400 transform -translate-y-1/2"></div>
                        <div className="absolute top-1/2 -right-32 w-32 h-0.5 bg-orange-400 transform -translate-y-1/2"></div>

                        {/* Connection nodes at the ends */}
                        <div className="absolute top-1/2 -left-32 w-3 h-3 bg-blue-500 rounded-full border border-blue-600 transform -translate-y-1/2 -translate-x-1"></div>
                        <div className="absolute top-1/2 -right-32 w-3 h-3 bg-purple-500 rounded-full border border-purple-600 transform -translate-y-1/2 translate-x-1"></div>
                      </div>
                    </div>
                  </div>

                  {/* Branch Labels */}
                  <div className="flex justify-center mb-6">
                    <div className="grid grid-cols-2 gap-32 text-center">
                      <div className="bg-blue-100 border border-blue-300 rounded-lg p-3">
                        <div className="flex items-center justify-center space-x-2">
                          <div className="w-3 h-3 bg-blue-500 rounded-full"></div>
                          <span className="font-medium text-blue-900">Hardware Path</span>
                        </div>
                        <p className="text-sm text-blue-700 mt-1">Physical deployment & data collection</p>
                      </div>
                      <div className="bg-purple-100 border border-purple-300 rounded-lg p-3">
                        <div className="flex items-center justify-center space-x-2">
                          <div className="w-3 h-3 bg-purple-500 rounded-full"></div>
                          <span className="font-medium text-purple-900">Direct Simulation</span>
                        </div>
                        <p className="text-sm text-purple-700 mt-1">Virtual testing & validation</p>
                      </div>
                    </div>
                  </div>

                  {/* Branched Flow with Connecting Nodes */}
                  <div className="grid grid-cols-1 lg:grid-cols-2 gap-8">
                    {/* Left Branch: Hardware → Data → Simulation */}
                    <div className="space-y-6">
                      <div className="space-y-4">
                        {/* Connection Node to Hardware */}
                        <div className="flex justify-center">
                          <div className="w-3 h-3 bg-blue-500 rounded-full border border-blue-600"></div>
                        </div>
                        <div className="flex justify-center">
                          <div className="w-0.5 h-4 bg-blue-400"></div>
                        </div>

                        {/* Hardware Card */}
                        <div className="flex justify-center">
                          <Card
                            className={`cursor-pointer transition-all duration-200 hover:shadow-lg w-64 ${
                              selectedStep === "hardware" ? "ring-2 ring-blue-500 shadow-lg" : ""
                            }`}
                            onClick={() => handleCardClick("hardware")}
                            title="Click to view details"
                          >
                            <CardContent className="p-6 text-center">
                              <div className="flex flex-col items-center space-y-3">
                                <HardDrive className="h-6 w-6" />
                                <CardTitle className="text-lg">Hardware Deployment</CardTitle>
                                <div className="flex items-center space-x-2">
                                  {getStatusIcon(workflowSteps.find((s) => s.id === "hardware")?.status || "pending")}
                                  <Badge
                                    className={getStatusColor(
                                      workflowSteps.find((s) => s.id === "hardware")?.status || "pending",
                                    )}
                                  >
                                    PENDING
                                  </Badge>
                                </div>
                                <div className="w-full">
                                  <div className="flex justify-between text-sm mb-1">
                                    <span>Progress:</span>
                                    <span>0%</span>
                                  </div>
                                  <Progress value={0} className="h-2" />
                                </div>
                              </div>
                            </CardContent>
                          </Card>
                        </div>

                        {/* Connection Node between Hardware and Data */}
                        <div className="flex justify-center">
                          <div className="w-0.5 h-4 bg-blue-400"></div>
                        </div>
                        <div className="flex justify-center">
                          <div className="w-3 h-3 bg-blue-500 rounded-full border border-blue-600"></div>
                        </div>
                        <div className="flex justify-center">
                          <div className="w-0.5 h-4 bg-blue-400"></div>
                        </div>

                        {/* Data Collection Card */}
                        <div className="flex justify-center">
                          <Card
                            className={`cursor-pointer transition-all duration-200 hover:shadow-lg w-64 ${
                              selectedStep === "data" ? "ring-2 ring-blue-500 shadow-lg" : ""
                            }`}
                            onClick={() => handleCardClick("data")}
                            title="Click to view details"
                          >
                            <CardContent className="p-6 text-center">
                              <div className="flex flex-col items-center space-y-3">
                                <Database className="h-6 w-6" />
                                <CardTitle className="text-lg">Data Collection</CardTitle>
                                <div className="flex items-center space-x-2">
                                  {getStatusIcon(workflowSteps.find((s) => s.id === "data")?.status || "pending")}
                                  <Badge
                                    className={getStatusColor(
                                      workflowSteps.find((s) => s.id === "data")?.status || "pending",
                                    )}
                                  >
                                    PENDING
                                  </Badge>
                                </div>
                                <div className="w-full">
                                  <div className="flex justify-between text-sm mb-1">
                                    <span>Progress:</span>
                                    <span>0%</span>
                                  </div>
                                  <Progress value={0} className="h-2" />
                                </div>
                              </div>
                            </CardContent>
                          </Card>
                        </div>

                        {/* Connection Node from Data to Simulation */}
                        <div className="flex justify-center">
                          <div className="w-0.5 h-4 bg-blue-400"></div>
                        </div>
                        <div className="flex justify-center">
                          <div className="w-3 h-3 bg-blue-500 rounded-full border border-blue-600"></div>
                        </div>
                      </div>
                    </div>

                    {/* Right Branch: Direct Simulation */}
                    <div className="space-y-6">
                      <div className="flex justify-center">
                        {/* Connection Node to Direct Simulation */}
                        <div className="w-3 h-3 bg-purple-500 rounded-full border border-purple-600"></div>
                      </div>
                      <div className="flex justify-center">
                        <div className="w-0.5 h-16 bg-purple-400"></div>
                      </div>
                      <div className="flex justify-center">
                        <div className="text-sm text-gray-600 bg-purple-50 p-4 rounded border-2 border-dashed border-purple-300 max-w-xs text-center">
                          <div className="flex items-center justify-center space-x-2 mb-2">
                            <div className="w-2 h-2 bg-purple-500 rounded-full"></div>
                            <span className="font-medium">Direct Path</span>
                          </div>
                          Control System feeds directly into Simulation for rapid prototyping and algorithm testing
                        </div>
                      </div>
                      <div className="flex justify-center">
                        <div className="w-0.5 h-16 bg-purple-400"></div>
                      </div>
                      <div className="flex justify-center">
                        {/* Connection Node from Direct Path to Simulation */}
                        <div className="w-3 h-3 bg-purple-500 rounded-full border border-purple-600"></div>
                      </div>
                    </div>
                  </div>

                  {/* Convergence to Simulation */}
                  <div className="mt-8">
                    <div className="flex justify-center mb-4">
                      {/* Convergence Node */}
                      <div className="relative">
                        <div className="w-6 h-6 bg-green-500 rounded-full border-2 border-green-600 flex items-center justify-center">
                          <Play className="h-3 w-3 text-white" />
                        </div>

                        {/* Connecting lines from both paths */}
                        <div className="absolute top-1/2 -left-32 w-32 h-0.5 bg-blue-400 transform -translate-y-1/2"></div>
                        <div className="absolute top-1/2 -right-32 w-32 h-0.5 bg-purple-400 transform -translate-y-1/2"></div>

                        {/* Connection nodes from both paths */}
                        <div className="absolute top-1/2 -left-32 w-3 h-3 bg-blue-500 rounded-full border border-blue-600 transform -translate-y-1/2 -translate-x-1"></div>
                        <div className="absolute top-1/2 -right-32 w-3 h-3 bg-purple-500 rounded-full border border-purple-600 transform -translate-y-1/2 translate-x-1"></div>
                      </div>
                    </div>

                    <div className="flex justify-center mb-4">
                      <div className="bg-green-100 border border-green-300 rounded-lg p-3">
                        <div className="flex items-center justify-center space-x-2">
                          <div className="w-3 h-3 bg-green-500 rounded-full"></div>
                          <span className="font-medium text-green-900">Paths Converge</span>
                        </div>
                        <p className="text-sm text-green-700 mt-1">
                          Both hardware and direct paths feed into simulation
                        </p>
                      </div>
                    </div>

                    {/* Connection line down to Simulation */}
                    <div className="flex justify-center">
                      <div className="w-0.5 h-6 bg-green-400"></div>
                    </div>

                    {/* Simulation Card */}
                    <div className="flex justify-center">
                      <Card
                        className={`cursor-pointer transition-all duration-200 hover:shadow-lg w-64 ${
                          selectedStep === "simulation" ? "ring-2 ring-blue-500 shadow-lg" : ""
                        } hover:ring-2 hover:ring-purple-300`}
                        onClick={() => handleCardClick("simulation")}
                        onDoubleClick={(e) => handleCardDoubleClick("simulation", e)}
                        title="Click to view details • Double-click for simulation dashboard"
                      >
                        <CardContent className="p-6 text-center">
                          <div className="flex flex-col items-center space-y-3">
                            <Play className="h-6 w-6" />
                            <CardTitle className="text-lg">Simulation</CardTitle>
                            <div className="flex items-center space-x-2">
                              {getStatusIcon(workflowSteps.find((s) => s.id === "simulation")?.status || "pending")}
                              <Badge
                                className={getStatusColor(
                                  workflowSteps.find((s) => s.id === "simulation")?.status || "pending",
                                )}
                              >
                                IN-PROGRESS
                              </Badge>
                            </div>
                            <div className="w-full">
                              <div className="flex justify-between text-sm mb-1">
                                <span>Progress:</span>
                                <span>60%</span>
                              </div>
                              <Progress value={60} className="h-2" />
                            </div>
                            <div className="text-xs text-purple-600 font-medium">Double-click for details</div>
                          </div>
                        </CardContent>
                      </Card>
                    </div>
                  </div>

                  {/* Final Export Step with Connection Node */}
                  <div className="mt-8">
                    <div className="flex justify-center mb-4">
                      <div className="w-0.5 h-6 bg-green-400"></div>
                    </div>
                    <div className="flex justify-center mb-4">
                      <div className="w-3 h-3 bg-green-500 rounded-full border border-green-600"></div>
                    </div>
                    <div className="flex justify-center mb-4">
                      <div className="w-0.5 h-6 bg-red-400"></div>
                    </div>

                    {/* MODIFIED: Export & Reports Card - Double-click opens Export Section */}
                    <div className="flex justify-center">
                      <Card
                        className={`cursor-pointer transition-all duration-200 hover:shadow-lg w-64 ${
                          selectedStep === "export" ? "ring-2 ring-blue-500 shadow-lg" : ""
                        } hover:ring-2 hover:ring-green-300`}
                        onClick={() => handleCardClick("export")}
                        onDoubleClick={(e) => handleCardDoubleClick("export", e)}
                        title="Click to view details • Double-click for Export & Reports section"
                      >
                        <CardContent className="p-6 text-center">
                          <div className="flex flex-col items-center space-y-3">
                            <div className="flex items-center space-x-2">
                              <FileText className="h-6 w-6" />
                              <Button
                                size="sm"
                                variant="ghost"
                                className="h-6 w-6 p-0 hover:bg-green-100"
                                onClick={(e) => {
                                  e.stopPropagation()
                                  downloadComprehensiveReport()
                                }}
                                title="Download Report Summary"
                              >
                                <Download className="h-3 w-3 text-green-600" />
                              </Button>
                            </div>
                            <CardTitle className="text-lg">Export & Reports</CardTitle>
                            <div className="flex items-center space-x-2">
                              {getStatusIcon(workflowSteps.find((s) => s.id === "export")?.status || "pending")}
                              <Badge
                                className={getStatusColor(
                                  workflowSteps.find((s) => s.id === "export")?.status || "pending",
                                )}
                              >
                                PENDING
                              </Badge>
                            </div>
                            <div className="w-full">
                              <div className="flex justify-between text-sm mb-1">
                                <span>Progress:</span>
                                <span>0%</span>
                              </div>
                              <Progress value={0} className="h-2" />
                            </div>
                            <div className="text-xs text-green-600 font-medium">Double-click for Export section</div>
                          </div>
                        </CardContent>
                      </Card>
                    </div>
                  </div>
                </div>

                {/* Flow Summary */}
                <Card className="mt-6 bg-gradient-to-r from-blue-50 to-purple-50 border-blue-200">
                  <CardHeader>
                    <CardTitle className="flex items-center space-x-2">
                      <GitBranch className="h-5 w-5 text-blue-600" />
                      <span>Complete Development Flow (PDF Accurate)</span>
                    </CardTitle>
                    <CardDescription>
                      Branching architecture with parallel hardware and simulation paths
                    </CardDescription>
                  </CardHeader>
                  <CardContent>
                    <div className="space-y-4">
                      {/* Linear Path */}
                      <div className="flex items-center justify-center text-sm">
                        <div className="flex items-center space-x-2">
                          <Box className="h-6 w-6 text-blue-600" />
                          <span>CAD</span>
                        </div>
                        <ArrowRight className="h-4 w-4 text-gray-400 mx-2" />
                        <div className="flex items-center space-x-2">
                          <Cog className="h-6 w-6 text-green-600" />
                          <span>Mechanical</span>
                        </div>
                        <ArrowRight className="h-4 w-4 text-gray-400 mx-2" />
                        <div className="flex items-center space-x-2">
                          <Settings className="h-6 w-6 text-orange-600" />
                          <span>Control</span>
                        </div>
                      </div>

                      {/* Branching Paths */}
                      <div className="grid grid-cols-1 md:grid-cols-2 gap-4 text-sm">
                        <div className="bg-blue-50 p-3 rounded border">
                          <div className="font-medium text-blue-900 mb-2">Hardware Path:</div>
                          <div className="flex items-center space-x-1">
                            <HardDrive className="h-4 w-4" />
                            <span>Hardware</span>
                            <ArrowRight className="h-3 w-3" />
                            <Database className="h-4 w-4" />
                            <span>Data</span>
                            <ArrowRight className="h-3 w-3" />
                            <Play className="h-4 w-4" />
                            <span>Simulation</span>
                          </div>
                        </div>
                        <div className="bg-purple-50 p-3 rounded border">
                          <div className="font-medium text-purple-900 mb-2">Direct Path:</div>
                          <div className="flex items-center space-x-1">
                            <Settings className="h-4 w-4" />
                            <span>Control</span>
                            <ArrowRight className="h-3 w-3" />
                            <Play className="h-4 w-4" />
                            <span>Simulation</span>
                          </div>
                        </div>
                      </div>

                      {/* Final Step */}
                      <div className="flex items-center justify-center text-sm">
                        <div className="flex items-center space-x-2">
                          <Play className="h-6 w-6 text-purple-600" />
                          <span>Simulation</span>
                        </div>
                        <ArrowRight className="h-4 w-4 text-gray-400 mx-2" />
                        <div className="flex items-center space-x-2">
                          <FileText className="h-6 w-6 text-red-600" />
                          <span>Export & Reports</span>
                        </div>
                      </div>
                    </div>

                    {/* LLM Integration Note */}
                    <div className="mt-4 p-3 bg-purple-50 rounded-lg border border-purple-200">
                      <div className="flex items-center space-x-2">
                        <Brain className="h-5 w-5 text-purple-600" />
                        <span className="font-medium text-purple-900">LLM Integration</span>
                      </div>
                      <p className="text-sm text-purple-700 mt-1">
                        The LLM Assistant (side panel) can interact with all workflow steps and integrates with GitHub,
                        Bitbucket, and JIRA for comprehensive project management.
                      </p>
                    </div>
                  </CardContent>
                </Card>

                {/* Selected Step Details */}
                {selectedStep && (
                  <Card className="mt-6">
                    <CardHeader>
                      <CardTitle className="flex items-center space-x-2">
                        {workflowSteps.find((s) => s.id === selectedStep)?.icon}
                        <span>{workflowSteps.find((s) => s.id === selectedStep)?.title} Details</span>
                      </CardTitle>
                    </CardHeader>
                    <CardContent>
                      <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
                        <div>
                          <h4 className="font-semibold mb-2">Description</h4>
                          <p className="text-gray-600 mb-4">
                            {workflowSteps.find((s) => s.id === selectedStep)?.description}
                          </p>

                          {/* Tools */}
                          <div>
                            <h4 className="font-medium mb-3">Available Tools:</h4>
                            <div className="flex flex-wrap gap-2 mb-3">
                              {selectedCardDetails?.tools.map((tool) => (
                                <Badge key={tool} variant="secondary" className="px-3 py-1">
                                  {tool}
                                </Badge>
                              ))}
                            </div>
                          </div>
                        </div>

                        <div>
                          <h4 className="font-semibold mb-2">Integrations</h4>
                          <div className="space-y-2">
                            {workflowSteps
                              .find((s) => s.id === selectedStep)
                              ?.integrations?.map((integration) => (
                                <div key={integration} className="flex items-center space-x-2">
                                  <GitBranch className="h-4 w-4 text-green-500" />
                                  <span className="text-sm">{integration}</span>
                                </div>
                              ))}
                          </div>

                          <div className="mt-4 space-y-2">
                            <Button className="w-full">Configure Step</Button>
                            <Button
                              variant="outline"
                              className="w-full bg-transparent"
                              onClick={() => setShowLLMPanel(true)}
                            >
                              <Brain className="h-4 w-4 mr-2" />
                              Get AI Assistance
                            </Button>
                          </div>
                        </div>
                      </div>
                    </CardContent>
                  </Card>
                )}
              </TabsContent>

              <TabsContent value="integrations" className="space-y-6">
                <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-4">
                  {integrationTools.map((tool) => (
                    <Card key={tool.name}>
                      <CardContent className="p-4">
                        <div className="flex items-center justify-between">
                          <div className="flex items-center space-x-2">
                            {tool.icon}
                            <span className="font-medium">{tool.name}</span>
                          </div>
                          <Badge
                            variant={tool.status === "connected" ? "default" : "secondary"}
                            className={tool.status === "connected" ? "bg-green-100 text-green-800" : ""}
                          >
                            {tool.status}
                          </Badge>
                        </div>
                      </CardContent>
                    </Card>
                  ))}
                </div>

                <Card>
                  <CardHeader>
                    <CardTitle>Integration Features</CardTitle>
                    <CardDescription>Key capabilities for version control and project management</CardDescription>
                  </CardHeader>
                  <CardContent>
                    <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
                      <div className="space-y-2">
                        <h4 className="font-semibold">Version Control</h4>
                        <ul className="text-sm text-gray-600 space-y-1">
                          <li>• Automated commits</li>
                          <li>• Branch management</li>
                          <li>• Merge conflict resolution</li>
                          <li>• Release tagging</li>
                        </ul>
                      </div>
                      <div className="space-y-2">
                        <h4 className="font-semibold">Compatibility</h4>
                        <ul className="text-sm text-gray-600 space-y-1">
                          <li>• Cross-platform support</li>
                          <li>• API handshaking</li>
                          <li>• Format validation</li>
                          <li>• Dependency management</li>
                        </ul>
                      </div>
                      <div className="space-y-2">
                        <h4 className="font-semibold">Project Management</h4>
                        <ul className="text-sm text-gray-600 space-y-1">
                          <li>• Issue tracking</li>
                          <li>• Sprint planning</li>
                          <li>• Progress monitoring</li>
                          <li>• Team collaboration</li>
                        </ul>
                      </div>
                    </div>
                  </CardContent>
                </Card>
              </TabsContent>

              <TabsContent value="monitoring" className="space-y-6">
                <div className="grid grid-cols-1 md:grid-cols-3 gap-6">
                  <Card>
                    <CardHeader>
                      <CardTitle className="flex items-center space-x-2">
                        <Monitor className="h-5 w-5" />
                        <span>System Status</span>
                      </CardTitle>
                    </CardHeader>
                    <CardContent>
                      <div className="space-y-3">
                        <div className="flex justify-between items-center">
                          <span className="text-sm">Overall Health</span>
                          <Badge className="bg-green-100 text-green-800">Healthy</Badge>
                        </div>
                        <div className="flex justify-between items-center">
                          <span className="text-sm">Active Processes</span>
                          <span className="text-sm font-medium">4/7</span>
                        </div>
                        <div className="flex justify-between items-center">
                          <span className="text-sm">Completion Rate</span>
                          <span className="text-sm font-medium">57%</span>
                        </div>
                      </div>
                    </CardContent>
                  </Card>

                  <Card>
                    <CardHeader>
                      <CardTitle>Recent Activity</CardTitle>
                    </CardHeader>
                    <CardContent>
                      <div className="space-y-2 text-sm">
                        <div className="flex items-center space-x-2">
                          <CheckCircle className="h-4 w-4 text-green-500" />
                          <span>CAD design completed</span>
                        </div>
                        <div className="flex items-center space-x-2">
                          <Clock className="h-4 w-4 text-blue-500" />
                          <span>Control system in progress</span>
                        </div>
                        <div className="flex items-center space-x-2">
                          <GitBranch className="h-4 w-4 text-gray-500" />
                          <span>GitHub sync successful</span>
                        </div>
                      </div>
                    </CardContent>
                  </Card>

                  <Card>
                    <CardHeader>
                      <CardTitle>Quick Actions</CardTitle>
                    </CardHeader>
                    <CardContent className="space-y-2">
                      <Button variant="outline" className="w-full justify-start bg-transparent">
                        <Play className="h-4 w-4 mr-2" />
                        Run Simulation
                      </Button>
                      <Button variant="outline" className="w-full justify-start bg-transparent">
                        <HardDrive className="h-4 w-4 mr-2" />
                        Deploy to Hardware
                      </Button>
                      <Button variant="outline" className="w-full justify-start bg-transparent">
                        <FileText className="h-4 w-4 mr-2" />
                        Generate Report
                      </Button>
                    </CardContent>
                  </Card>
                </div>
              </TabsContent>
            </Tabs>
          </div>
        </div>

        {/* Card Details Modal */}
        <Dialog open={showCardDetails} onOpenChange={setShowCardDetails}>
          <DialogContent className="max-w-2xl">
            <DialogHeader>
              <DialogTitle className="flex items-center space-x-2">
                {selectedCardDetails?.icon}
                <span>{selectedCardDetails?.title}</span>
              </DialogTitle>
              <DialogDescription>{selectedCardDetails?.description}</DialogDescription>
            </DialogHeader>

            <div className="space-y-6">
              {/* Status and Progress */}
              <div className="space-y-4">
                <div className="flex items-center justify-between">
                  <span className="font-medium">Status:</span>
                  <Badge className={`${getStatusColor(selectedCardDetails?.status || "pending")}`}>
                    {selectedCardDetails?.status?.replace("-", " ").toUpperCase()}
                  </Badge>
                </div>

                <div>
                  <div className="flex justify-between text-sm mb-2">
                    <span className="font-medium">Progress:</span>
                    <span>{selectedCardDetails?.progress}%</span>
                  </div>
                  <Progress value={selectedCardDetails?.progress || 0} className="h-3" />
                </div>
              </div>

              {/* Tools */}
              <div>
                <h4 className="font-medium mb-3">Available Tools:</h4>
                <div className="flex flex-wrap gap-2 mb-3">
                  {selectedCardDetails?.tools.map((tool) => (
                    <Badge key={tool} variant="secondary" className="px-3 py-1">
                      {tool}
                    </Badge>
                  ))}
                </div>
                {/* ONLY show View Comprehensive Report button for Export & Reports card */}
                {selectedCardDetails?.id === "export" && (
                  <Button
                    variant="outline"
                    size="sm"
                    className="w-full bg-transparent"
                    onClick={() => {
                      setShowReportsModal(true)
                      setShowCardDetails(false)
                    }}
                  >
                    <FileBarChart className="h-4 w-4 mr-2" />
                    View Comprehensive Report
                  </Button>
                )}
              </div>

              {/* Integrations */}
              {selectedCardDetails?.integrations && (
                <div>
                  <h4 className="font-medium mb-3">Integrations:</h4>
                  <div className="space-y-2">
                    {selectedCardDetails.integrations.map((integration) => (
                      <div key={integration} className="flex items-center space-x-2">
                        <GitBranch className="h-4 w-4 text-green-500" />
                        <span className="text-sm">{integration}</span>
                      </div>
                    ))}
                  </div>
                </div>
              )}

              {/* Action Buttons */}
              <div className="flex space-x-3 pt-4">
                <Button className="flex-1">Configure Step</Button>
                <Button
                  variant="outline"
                  className="flex-1 bg-transparent"
                  onClick={() => {
                    setShowLLMPanel(true)
                    setShowCardDetails(false)
                  }}
                >
                  <Brain className="h-4 w-4 mr-2" />
                  Get AI Assistance
                </Button>
              </div>
            </div>
          </DialogContent>
        </Dialog>

        {/* LLM Side Panel */}
        <div
          className={`fixed right-0 top-0 h-full w-96 bg-white border-l border-gray-200 shadow-lg transform transition-transform duration-300 z-50 ${showLLMPanel ? "translate-x-0" : "translate-x-full"}`}
        >
          <div className="flex flex-col h-full">
            {/* Panel Header */}
            <div className="p-4 border-b border-gray-200 bg-gradient-to-r from-purple-50 to-blue-50">
              <div className="flex items-center justify-between">
                <div className="flex items-center space-x-2">
                  <Brain className="h-6 w-6 text-purple-600" />
                  <h2 className="text-lg font-semibold">LLM Assistant</h2>
                </div>
                <Button variant="ghost" size="sm" onClick={() => setShowLLMPanel(false)} className="h-8 w-8 p-0">
                  <ChevronRight className="h-4 w-4" />
                </Button>
              </div>

              {/* VLA Model Selection */}
              <div className="mt-3">
                <Select value={selectedVLAModel} onValueChange={setSelectedVLAModel}>
                  <SelectTrigger className="w-full">
                    <SelectValue />
                  </SelectTrigger>
                  <SelectContent>
                    {vlaModels.map((model) => (
                      <SelectItem key={model.id} value={model.id}>
                        <div>
                          <div className="font-medium text-sm">{model.name}</div>
                          <div className="text-xs text-gray-500">{model.description}</div>
                        </div>
                      </SelectItem>
                    ))}
                  </SelectContent>
                </Select>
              </div>

              {/* Model Capabilities */}
              <div className="flex flex-wrap gap-1 mt-2">
                {vlaModels
                  .find((m) => m.id === selectedVLAModel)
                  ?.capabilities.map((capability) => (
                    <Badge key={capability} variant="secondary" className="text-xs flex items-center space-x-1">
                      {capability === "Vision" && <Eye className="h-2 w-2" />}
                      {capability === "Language" && <FileText className="h-2 w-2" />}
                      {capability === "Action" && <Lightning className="h-2 w-2" />}
                      {capability === "Code Generation" && <Code className="h-2 w-2" />}
                      <span>{capability}</span>
                    </Badge>
                  ))}
              </div>
            </div>

            {/* Chat Messages */}
            <div className="flex-1 overflow-hidden">
              <ScrollArea className="h-full p-4">
                <div className="space-y-4">
                  {chatMessages.map((message) => (
                    <div
                      key={message.id}
                      className={`flex ${message.type === "user" ? "justify-end" : "justify-start"}`}
                    >
                      <div
                        className={`max-w-[85%] rounded-lg p-3 ${
                          message.type === "user" ? "bg-blue-500 text-white" : "bg-gray-100 text-gray-900"
                        }`}
                      >
                        {message.imageUrl && (
                          <img
                            src={message.imageUrl || "/placeholder.svg"}
                            alt="Uploaded"
                            className="max-w-32 rounded mb-2"
                          />
                        )}
                        <div className="text-sm">{message.content}</div>
                        {message.codeSnippet && (
                          <div className="mt-2 bg-gray-800 text-green-400 p-2 rounded text-xs font-mono relative">
                            <Button
                              size="sm"
                              variant="ghost"
                              className="absolute top-1 right-1 h-5 w-5 p-0 text-gray-400 hover:text-white"
                              onClick={() => copyToClipboard(message.codeSnippet!)}
                            >
                              <Copy className="h-2 w-2" />
                            </Button>
                            <pre className="whitespace-pre-wrap text-xs">{message.codeSnippet}</pre>
                          </div>
                        )}
                        {message.relatedStep && (
                          <div className="text-xs opacity-70 mt-1">
                            Related to: {workflowSteps.find((s) => s.id === message.relatedStep)?.title}
                          </div>
                        )}
                        <div className="text-xs opacity-70 mt-1">{message.timestamp}</div>
                      </div>
                    </div>
                  ))}
                  {isLoading && (
                    <div className="flex justify-start">
                      <div className="bg-gray-100 rounded-lg p-3">
                        <div className="flex items-center space-x-2">
                          <RefreshCw className="h-4 w-4 animate-spin" />
                          <span className="text-sm">Analyzing...</span>
                        </div>
                      </div>
                    </div>
                  )}
                </div>
              </ScrollArea>
            </div>

            {/* Quick Prompts */}
            <div className="p-3 border-t border-gray-100">
              <h4 className="font-medium text-sm mb-2">Quick Prompts</h4>
              <div className="space-y-1 max-h-24 overflow-y-auto">
                {quickPrompts.slice(0, 3).map((prompt, index) => (
                  <Button
                    key={index}
                    variant="outline"
                    size="sm"
                    className="w-full text-left justify-start h-auto p-2 text-xs bg-transparent"
                    onClick={() => handleQuickPrompt(prompt)}
                  >
                    {prompt}
                  </Button>
                ))}
              </div>
            </div>

            {/* Input Area */}
            <div className="p-4 border-t border-gray-200">
              {uploadedImage && (
                <div className="flex items-center space-x-2 p-2 bg-gray-50 rounded mb-2">
                  <ImageIcon className="h-4 w-4" />
                  <span className="text-sm">Image uploaded</span>
                  <Button size="sm" variant="ghost" onClick={() => setUploadedImage(null)}>
                    <X className="h-3 w-3" />
                  </Button>
                </div>
              )}

              <div className="flex space-x-2">
                <div className="flex-1">
                  <Textarea
                    placeholder={`Ask about ${workflowSteps.find((s) => s.id === selectedStep)?.title || "your workflow"}...`}
                    value={currentMessage}
                    onChange={(e) => setCurrentMessage(e.target.value)}
                    onKeyDown={(e) => {
                      if (e.key === "Enter" && !e.shiftKey) {
                        e.preventDefault()
                        handleSendMessage()
                      }
                    }}
                    className="min-h-[60px] text-sm"
                  />
                </div>
                <div className="flex flex-col space-y-1">
                  <input
                    type="file"
                    accept="image/*"
                    onChange={handleImageUpload}
                    className="hidden"
                    id="image-upload"
                  />
                  <Button
                    size="sm"
                    variant="outline"
                    onClick={() => document.getElementById("image-upload")?.click()}
                    className="bg-transparent h-8 w-8 p-0"
                  >
                    <Upload className="h-3 w-3" />
                  </Button>
                  <Button
                    size="sm"
                    onClick={handleSendMessage}
                    disabled={!currentMessage.trim() && !uploadedImage}
                    className="h-8 w-8 p-0"
                  >
                    <Send className="h-3 w-3" />
                  </Button>
                </div>
              </div>

              {/* Current Step Context */}
              <div className="bg-blue-50 p-2 rounded mt-2">
                <div className="text-xs text-blue-900 font-medium">
                  Current Focus: {workflowSteps.find((s) => s.id === selectedStep)?.title}
                </div>
                <div className="text-xs text-blue-700">
                  Progress: {workflowSteps.find((s) => s.id === selectedStep)?.progress}% • Status:{" "}
                  {workflowSteps.find((s) => s.id === selectedStep)?.status}
                </div>
              </div>
            </div>
          </div>
        </div>

        {/* LLM Panel Toggle Button */}
        {!showLLMPanel && (
          <Button
            className="fixed right-4 top-1/2 transform -translate-y-1/2 z-40 rounded-l-lg rounded-r-none bg-purple-600 hover:bg-purple-700"
            onClick={() => setShowLLMPanel(true)}
          >
            <div className="flex flex-col items-center space-y-1">
              <Brain className="h-5 w-5" />
              <span className="text-xs">AI</span>
            </div>
          </Button>
        )}
      </div>
    </div>
  )
}