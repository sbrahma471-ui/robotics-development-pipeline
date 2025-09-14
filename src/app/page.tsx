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
  Cpu,
  Plus,
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
  features?: string[]
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

// CORRECTED workflow order to match the enhanced PDF diagram with detailed features
const workflowSteps: WorkflowStep[] = [
  {
    id: "mechanical",
    title: "Mechanical Design",
    description: "CAD integration, URDF/XML management with LLM assistance",
    icon: <Box className="h-6 w-6" />,
    status: "completed",
    progress: 100,
    tools: ["CAD/Fusion360", "URDF Creator", "XML Editor", "LLM Prompt Assistant"],
    integrations: ["GitHub", "Version Control"],
    features: [
      "Integrate CAD/Fusion 360",
      "Directly Create or Import URDF/xml",
      "Use LLM Prompt for design assistance",
      "Export URDF/xml files"
    ]
  },
  {
    id: "control",
    title: "Build Control System", 
    description: "Multi-approach controller development with simulation compatibility",
    icon: <Settings className="h-6 w-6" />,
    status: "in-progress",
    progress: 70,
    tools: ["Code Editor", "Controller Import", "GUI Builder", "Format Converter"],
    integrations: ["GitHub", "JIRA"],
    branches: ["simulation"],
    features: [
      "Build with code",
      "Import Existing Controller", 
      "Build with GUI",
      "Convert to Simulation Compatible Format"
    ]
  },
  {
    id: "simulation",
    title: "Simulation",
    description: "Multi-physics engine testing with automated reporting",
    icon: <Play className="h-6 w-6" />,
    status: "in-progress", 
    progress: 60,
    tools: ["Genesis AI", "Isaac Sim", "Gazebo", "Cloud Platform"],
    integrations: ["JIRA", "GitHub"],
    features: [
      "Integrate Physics Engine (Genesis AI, Isaac Sim, Gazebo, etc.)",
      "Setup the Simulation Environment", 
      "Import the Robot",
      "Integrate the Controller in the Robot",
      "Define Task and Setup Reward Function",
      "Run Simulation in Local System/Cloud",
      "Train Controller",
      "Automated Report Generation",
      "Export Firmware for the Robot"
    ]
  },
  {
    id: "hardware",
    title: "Hardware Training",
    description: "Physical robot deployment and real-world testing",
    icon: <Cpu className="h-6 w-6" />,
    status: "pending",
    progress: 0,
    tools: ["Robot Hardware", "Sensor Integration", "Actuator Control", "Safety Monitoring"],
    integrations: ["Hardware APIs", "Sensor Networks", "Cloud Platform"],
    features: [
      "Deploy trained model to physical robot",
      "Hardware sensor integration and calibration",
      "Real-world environment testing",
      "Performance validation against simulation",
      "Safety monitoring and emergency protocols",
      "Data collection from physical trials",
      "Hardware-software optimization",
      "Real-time feedback and adjustments"
    ]
  },
  {
    id: "export",
    title: "Export & Reports",
    description: "Comprehensive output generation and documentation",
    icon: <FileText className="h-6 w-6" />,
    status: "pending",
    progress: 0,
    tools: ["Report Generator", "File Exporter", "Documentation", "Data Analyzer"],
    integrations: ["JIRA", "Confluence"],
    features: [
      "Mechanical Design Files",
      "Robot Firmwares", 
      "Simulation Test Report",
      "Hardware Training Results",
      "Real-time Data Export",
      "Automated Documentation"
    ]
  },
]

const simulationEnvironments: SimulationEnvironment[] = [
  { name: "Genesis AI", status: "active", progress: 75, instances: 3 },
  { name: "Isaac Sim", status: "active", progress: 45, instances: 2 },
  { name: "Gazebo", status: "idle", progress: 0, instances: 0 },
  { name: "Cloud Platform", status: "active", progress: 90, instances: 1 },
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

interface OutputCard {
  id: string
  title: string
  description: string
  icon: React.ReactNode
  sourceModule: string
  outputType: string
  files: string[]
  status: "available" | "generating" | "pending"
  lastGenerated?: string
  fileSize?: string
}

const outputCards: OutputCard[] = [
  {
    id: "urdf-xml",
    title: "URDF/XML Files",
    description: "Exported mechanical design files from CAD integration",
    icon: <FileText className="h-6 w-6" />,
    sourceModule: "Mechanical Design",
    outputType: "Design Files",
    files: ["robot_model.urdf", "robot_description.xml", "materials.xml", "joints_config.yaml"],
    status: "available",
    lastGenerated: "2025-09-13 14:30:15",
    fileSize: "2.4 MB"
  },
  {
    id: "robot-firmware",
    title: "Robot Firmwares",
    description: "Compiled firmwares from simulation training completion",
    icon: <Cpu className="h-6 w-6" />,
    sourceModule: "Simulation",
    outputType: "Firmware Files",
    files: ["controller_v2.1.hex", "sensor_drivers.bin", "motion_planner.elf", "safety_monitor.firmware"],
    status: "available",
    lastGenerated: "2025-09-13 14:25:42",
    fileSize: "8.7 MB"
  },
  {
    id: "simulation-report",
    title: "Simulation Test Report",
    description: "Automated performance and test analysis from simulation",
    icon: <FileBarChart className="h-6 w-6" />,
    sourceModule: "Simulation",
    outputType: "Performance Report",
    files: ["simulation_report.pdf", "test_metrics.json", "performance_charts.png", "error_analysis.csv"],
    status: "available",
    lastGenerated: "2025-09-13 14:32:08",
    fileSize: "15.2 MB"
  },
  {
    id: "hardware-data",
    title: "Hardware Test Data",
    description: "Real-time data and hardware test results from physical trials",
    icon: <Database className="h-6 w-6" />,
    sourceModule: "Hardware Training",
    outputType: "Test Data & Reports",
    files: ["hardware_tests.json", "sensor_logs.csv", "real_time_data.log", "hardware_report.pdf"],
    status: "pending",
    lastGenerated: undefined,
    fileSize: undefined
  }
]

interface UrdfTemplate {
  id: string
  name: string
  description: string
  category: string
  content: string
}

const urdfTemplates: UrdfTemplate[] = [
  {
    id: "blank",
    name: "Blank URDF",
    description: "Start with an empty URDF template",
    category: "Basic",
    content: `<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Robot description goes here -->
  
  <!-- Example link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.166" ixy="0" ixz="0" iyy="0.166" iyz="0" izz="0.166"/>
    </inertial>
  </link>
  
</robot>`
  },
  {
    id: "robotic_arm",
    name: "6-DOF Robotic Arm",
    description: "Standard 6 degree-of-freedom robotic arm",
    category: "Manipulator",
    content: `<?xml version="1.0"?>
<robot name="robotic_arm">
  
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.1"/>
      </geometry>
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.02"/>
    </inertial>
  </link>

  <!-- Link 1 -->
  <link name="link_1">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.008" ixy="0" ixz="0" iyy="0.008" iyz="0" izz="0.0025"/>
    </inertial>
  </link>

  <!-- Joint 1 -->
  <joint name="joint_1" type="revolute">
    <parent link="base_link"/>
    <child link="link_1"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14159" upper="3.14159" effort="100" velocity="2"/>
  </joint>

  <!-- Add more links and joints as needed -->
  
</robot>`
  },
  {
    id: "mobile_robot",
    name: "Mobile Robot Base",
    description: "Wheeled mobile robot platform",
    category: "Mobile",
    content: `<?xml version="1.0"?>
<robot name="mobile_robot">
  
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.08" ixy="0" ixz="0" iyy="0.21" iyz="0" izz="0.25"/>
    </inertial>
  </link>

  <!-- Left Wheel -->
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.0025" ixy="0" ixz="0" iyy="0.0025" iyz="0" izz="0.005"/>
    </inertial>
  </link>

  <!-- Left Wheel Joint -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.175 -0.05" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- Right Wheel -->
  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.0025" ixy="0" ixz="0" iyy="0.0025" iyz="0" izz="0.005"/>
    </inertial>
  </link>

  <!-- Right Wheel Joint -->
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.175 -0.05" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
  
</robot>`
  },
  {
    id: "quadruped",
    name: "Quadruped Robot",
    description: "Four-legged robot template",
    category: "Legged",
    content: `<?xml version="1.0"?>
<robot name="quadruped_robot">
  
  <!-- Base Body -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.4 0.2 0.1"/>
      </geometry>
      <material name="brown">
        <color rgba="0.6 0.4 0.2 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.4 0.2 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="3.0"/>
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.08" iyz="0" izz="0.09"/>
    </inertial>
  </link>

  <!-- Front Left Leg -->
  <link name="front_left_leg">
    <visual>
      <geometry>
        <cylinder radius="0.02" length="0.2"/>
      </geometry>
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.02" length="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.0007" ixy="0" ixz="0" iyy="0.0007" iyz="0" izz="0.00004"/>
    </inertial>
  </link>

  <!-- Front Left Hip Joint -->
  <joint name="front_left_hip" type="revolute">
    <parent link="base_link"/>
    <child link="front_left_leg"/>
    <origin xyz="0.15 0.08 -0.05" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="5"/>
  </joint>

  <!-- Add more legs following similar pattern -->
  
</robot>`
  }
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
  const [showOutputDetails, setShowOutputDetails] = useState(false)
  const [selectedOutputCard, setSelectedOutputCard] = useState<OutputCard | null>(null)
  const [showUrdfEditor, setShowUrdfEditor] = useState(false)
  const [urdfContent, setUrdfContent] = useState("")
  const [selectedUrdfTemplate, setSelectedUrdfTemplate] = useState("blank")
  const [urdfFileName, setUrdfFileName] = useState("robot_model.urdf")
  const [isUrdfValid, setIsUrdfValid] = useState(true)
  const [urdfValidationErrors, setUrdfValidationErrors] = useState<string[]>([])
  const [showUrdfPreview, setShowUrdfPreview] = useState(false)

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

  // VS Code integration state variables
  const [mechanicalWorkspacePath] = useState("mechanical_design")
  const [currentEditingFile, setCurrentEditingFile] = useState<string | null>(null)
  const [isFileBeingEdited, setIsFileBeingEdited] = useState(false)
  const [lastModifiedTime, setLastModifiedTime] = useState<Date | null>(null)
  const [vscodeWatcher, setVscodeWatcher] = useState<NodeJS.Timeout | null>(null)
  const [isVSCodeOpen, setIsVSCodeOpen] = useState(false)
  const [openWithVSCode, setOpenWithVSCode] = useState(true)
  
  // Track created/imported files in mechanical design
  const [mechanicalFiles, setMechanicalFiles] = useState<Array<{
    name: string;
    type: string;
    createdAt: string;
    size?: string;
  }>>([])

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

    // Removed mechanical design double-click - now handled via button in card details
    if (stepId === "simulation") {
      console.log("Opening simulation details modal") // Debug log
      setShowSimulationDetails(true)
    } else if (stepId === "export") {
      console.log("Opening Export & Reports section") // Debug log
      setShowExportSection(true)
    }
  }

  const handleOutputCardClick = (outputCard: OutputCard) => {
    setSelectedOutputCard(outputCard)
    setShowOutputDetails(true)
  }

  const getOutputStatusColor = (status: string) => {
    switch (status) {
      case "available":
        return "bg-green-100 text-green-800"
      case "generating":
        return "bg-yellow-100 text-yellow-800"
      case "pending":
        return "bg-gray-100 text-gray-800"
      default:
        return "bg-gray-100 text-gray-800"
    }
  }

  const getOutputStatusIcon = (status: string) => {
    switch (status) {
      case "available":
        return <CheckCircle className="h-4 w-4 text-green-600" />
      case "generating":
        return <Clock className="h-4 w-4 text-yellow-600" />
      case "pending":
        return <AlertCircle className="h-4 w-4 text-gray-600" />
      default:
        return <AlertCircle className="h-4 w-4 text-gray-600" />
    }
  }

  // URDF Editor Functions
  const validateUrdf = (content: string): { isValid: boolean; errors: string[] } => {
    const errors: string[] = []
    
    // Basic XML validation
    try {
      const parser = new DOMParser()
      const xmlDoc = parser.parseFromString(content, "text/xml")
      const parseError = xmlDoc.getElementsByTagName("parsererror")
      
      if (parseError.length > 0) {
        errors.push("Invalid XML syntax")
      }
    } catch (error) {
      errors.push("XML parsing failed")
    }

    // URDF-specific validation
    if (!content.includes("<robot")) {
      errors.push("Missing <robot> root element")
    }
    
    if (!content.includes('name="')) {
      errors.push("Robot must have a name attribute")
    }

    // Check for required elements
    const requiredElements = ["link", "joint"]
    requiredElements.forEach(element => {
      if (!content.includes(`<${element}`)) {
        errors.push(`Missing required element: <${element}>`)
      }
    })

    return {
      isValid: errors.length === 0,
      errors
    }
  }

  const loadUrdfTemplate = (templateId: string) => {
    const template = urdfTemplates.find(t => t.id === templateId)
    if (template) {
      setUrdfContent(template.content)
      setSelectedUrdfTemplate(templateId)
      setUrdfFileName(`${template.name.toLowerCase().replace(/\s+/g, '_')}.urdf`)
      
      // Validate the template
      const validation = validateUrdf(template.content)
      setIsUrdfValid(validation.isValid)
      setUrdfValidationErrors(validation.errors)
    }
  }

  // File import handler
  const handleUrdfImport = (event: React.ChangeEvent<HTMLInputElement>) => {
    const file = event.target.files?.[0]
    if (file) {
      const reader = new FileReader()
      reader.onload = (e) => {
        const content = e.target?.result as string
        setUrdfContent(content)
        setUrdfFileName(file.name)
        
        // Validate the imported content
        const validation = validateUrdf(content)
        setIsUrdfValid(validation.isValid)
        setUrdfValidationErrors(validation.errors)
      }
      reader.readAsText(file)
    }
  }

  // Download URDF file
  const downloadUrdf = () => {
    if (!urdfContent) {
      alert('❌ No content to download. Please select a template or import a file first.')
      return
    }

    const blob = new Blob([urdfContent], { type: 'text/xml' })
    const url = URL.createObjectURL(blob)
    const a = document.createElement('a')
    a.href = url
    a.download = urdfFileName
    a.style.display = 'none'
    document.body.appendChild(a)
    a.click()
    document.body.removeChild(a)
    URL.revokeObjectURL(url)
  }

  const createFileAndOpenInVSCode = async (content: string, filename: string) => {
    try {
      const workspacePath = `c:\\Users\\Ritu\\robotics-workflow\\${mechanicalWorkspacePath}`
      const filePath = `${workspacePath}\\${filename}`
      
      console.log(`Creating file: ${filePath}`)
      console.log(`Content length: ${content.length} characters`)
      
      // Set current editing file state
      setCurrentEditingFile(filename)
      setIsFileBeingEdited(true)
      setLastModifiedTime(new Date())
      
      // Check if we're in Electron environment with our custom APIs
      const isElectron = typeof window !== 'undefined' && (window as any).electronAPI
      
      if (isElectron) {
        try {
          console.log('Using Electron APIs for file operations and VS Code opening...')
          
          // Use Electron APIs to write file and open VS Code
          const writeResult = (window as any).electronAPI.writeFile(filePath, content)
          
          if (writeResult.success) {
            console.log('File written successfully, opening in VS Code...')
            
            // Open in VS Code using Electron API
            const vscodeResult = await (window as any).electronAPI.openInVSCode(filePath)
            
            if (vscodeResult.success) {
              alert(`🚀 Success! File "${filename}" created and opened in VS Code!\n\n📁 Location: ${filePath}\n🔧 Method: ${vscodeResult.method}\n\n✏️ Edit the file in VS Code, then use "Re-import from VS Code" to bring changes back to the editor.`)
            } else {
              alert(`✅ File created successfully: ${filePath}\n\n❌ VS Code opening failed: ${vscodeResult.error}\n\n🔧 Please manually open VS Code with: code "${filePath}"`)
            }
          } else {
            throw new Error(`File write failed: ${writeResult.error}`)
          }
          
        } catch (electronError) {
          console.error('Electron API failed:', electronError)
          fallbackDownload(content, filename, filePath)
        }
      } else {
        console.log('Browser environment detected, using download fallback...')
        fallbackDownload(content, filename, filePath)
      }
      
    } catch (error) {
      console.error('Error creating file and opening VS Code:', error)
      alert('❌ Error creating file. Please check console for details.')
    }
  }

  const fallbackDownload = (content: string, filename: string, filePath: string) => {
    // Browser fallback - download file
    const blob = new Blob([content], { type: 'text/xml' })
    const url = URL.createObjectURL(blob)
    const a = document.createElement('a')
    a.href = url
    a.download = filename
    a.style.display = 'none'
    document.body.appendChild(a)
    a.click()
    document.body.removeChild(a)
    URL.revokeObjectURL(url)
    
    showDownloadInstructions(filename, filePath)
  }

  const showDownloadInstructions = (filename: string, filePath: string) => {
    alert(`📁 File "${filename}" has been downloaded and is ready for VS Code!\n\n🔧 Next steps:\n\n1. Save the downloaded file to:\n   ${filePath}\n\n2. Open in VS Code:\n   code "${filePath}"\n\n3. After editing, use "Re-import from VS Code" in the editor to bring your changes back!`)
  }

  const openInVSCodeEditor = async () => {
    if (!urdfContent) {
      alert('❌ No content to edit. Please select a template or import a file first.')
      return
    }

    try {
      const workspacePath = `c:\\Users\\Ritu\\robotics-workflow\\${mechanicalWorkspacePath}`
      const filePath = `${workspacePath}\\${urdfFileName}`
      
      console.log(`Opening in VS Code Editor: ${filePath}`)
      
      // Check if we're in Electron environment with our custom APIs
      const isElectron = typeof window !== 'undefined' && (window as any).electronAPI
      
      if (isElectron) {
        try {
          // Write current content to file
          const writeResult = (window as any).electronAPI.writeFile(filePath, urdfContent)
          
          if (writeResult.success) {
            // Open in VS Code
            const vscodeResult = await (window as any).electronAPI.openInVSCode(filePath)
            
            if (vscodeResult.success) {
              setCurrentEditingFile(urdfFileName)
              setIsFileBeingEdited(true)
              setIsVSCodeOpen(true)
              setLastModifiedTime(new Date())
              
              // Start watching for file changes and VS Code close
              startVSCodeWatcher(filePath)
              
              alert(`� VS Code opened with "${urdfFileName}"!\n\n✏️ Edit the file in VS Code. When you close VS Code, changes will be automatically saved back to the editor.\n\n📁 File location: ${filePath}`)
              
            } else {
              alert(`✅ File saved: ${filePath}\n\n❌ VS Code opening failed: ${vscodeResult.error}\n\n🔧 Please manually open VS Code: code "${filePath}"`)
            }
          } else {
            throw new Error(`File write failed: ${writeResult.error}`)
          }
          
        } catch (electronError) {
          console.error('Electron API failed:', electronError)
          fallbackVSCodeOpen()
        }
      } else {
        fallbackVSCodeOpen()
      }
      
    } catch (error) {
      console.error('Error opening VS Code editor:', error)
      alert('❌ Error opening VS Code editor. Please check console for details.')
    }
  }

  const fallbackVSCodeOpen = () => {
    // Browser fallback - download file and show instructions
    const blob = new Blob([urdfContent], { type: 'text/xml' })
    const url = URL.createObjectURL(blob)
    const a = document.createElement('a')
    a.href = url
    a.download = urdfFileName
    a.style.display = 'none'
    document.body.appendChild(a)
    a.click()
    document.body.removeChild(a)
    URL.revokeObjectURL(url)
    
    alert(`📁 File "${urdfFileName}" downloaded for VS Code editing!\n\n🔧 Manual steps:\n1. Save to: c:\\Users\\Ritu\\robotics-workflow\\mechanical_design\\${urdfFileName}\n2. Open: code "c:\\Users\\Ritu\\robotics-workflow\\mechanical_design\\${urdfFileName}"\n3. After editing, use "Re-import from VS Code" button`)
  }

  const startVSCodeWatcher = (filePath: string) => {
    // Clear any existing watcher
    if (vscodeWatcher) {
      clearInterval(vscodeWatcher)
    }

    // Check for file changes every 2 seconds
    const watcher = setInterval(async () => {
      const isElectron = typeof window !== 'undefined' && (window as any).electronAPI
      
      if (isElectron) {
        try {
          const readResult = (window as any).electronAPI.readFile(filePath)
          
          if (readResult.success) {
            // Check if content has changed
            if (readResult.content !== urdfContent) {
              console.log('File content changed, updating editor...')
              
              // Update the editor with new content
              setUrdfContent(readResult.content)
              
              // Validate the updated content
              const validation = validateUrdf(readResult.content)
              setIsUrdfValid(validation.isValid)
              setUrdfValidationErrors(validation.errors)
              
              setLastModifiedTime(new Date())
              
              // Show a subtle notification (could be replaced with a toast)
              console.log(`📝 File updated: ${readResult.content.length} characters, Valid: ${validation.isValid}`)
            }
          }
        } catch (error) {
          console.error('File watching error:', error)
        }
      }
    }, 2000) // Check every 2 seconds

    setVscodeWatcher(watcher)
    
    // Auto-stop watching after 30 minutes to prevent memory leaks
    setTimeout(() => {
      if (watcher) {
        clearInterval(watcher)
        setVscodeWatcher(null)
        setIsVSCodeOpen(false)
        console.log('VS Code file watching stopped after 30 minutes')
      }
    }, 30 * 60 * 1000)
  }

  const stopVSCodeWatcher = () => {
    if (vscodeWatcher) {
      clearInterval(vscodeWatcher)
      setVscodeWatcher(null)
    }
    setIsVSCodeOpen(false)
    setIsFileBeingEdited(false)
  }

  const handleUrdfContentChange = (content: string) => {
    setUrdfContent(content)
    
    // Real-time validation
    const validation = validateUrdf(content)
    setIsUrdfValid(validation.isValid)
    setUrdfValidationErrors(validation.errors)
  }

  const handleFileImport = async (event: React.ChangeEvent<HTMLInputElement>) => {
    const file = event.target.files?.[0]
    if (file) {
      const reader = new FileReader()
      reader.onload = async (e) => {
        const content = e.target?.result as string
        setUrdfContent(content)
        setUrdfFileName(file.name)
        setSelectedUrdfTemplate("imported")
        
        const validation = validateUrdf(content)
        setIsUrdfValid(validation.isValid)
        setUrdfValidationErrors(validation.errors)

        // Add imported file to mechanical files list
        const importedFile = {
          name: file.name,
          type: "URDF/XML (Imported)",
          createdAt: new Date().toLocaleString(),
          size: `${Math.round(file.size / 1024)}KB`
        }
        
        setMechanicalFiles(prev => {
          // Check if file already exists
          const exists = prev.some(f => f.name === file.name)
          if (exists) {
            // Update existing file
            return prev.map(f => f.name === file.name ? importedFile : f)
          } else {
            // Add new file
            return [...prev, importedFile]
          }
        })

        // Show success message
        alert(`✅ Successfully imported "${file.name}"!\n\n📁 File is now available in the Mechanical Design workspace.`)

        // Automatically open imported file in VS Code if enabled
        if (openWithVSCode) {
          await createFileAndOpenInVSCode(content, file.name)
        }
      }
      reader.readAsText(file)
    }
  }

  const saveToMechanicalWorkspace = async () => {
    try {
      // Download the URDF file for manual workspace addition
      downloadUrdf()
      
      // Add file to mechanical files list
      const newFile = {
        name: urdfFileName,
        type: "URDF/XML",
        createdAt: new Date().toLocaleString(),
        size: `${Math.round(urdfContent.length / 1024)}KB`
      }
      
      setMechanicalFiles(prev => [...prev, newFile])
      
      // Show success message
      alert(`🎉 URDF file "${urdfFileName}" has been saved to the mechanical design workspace!\n\n✅ Mechanical Design step marked as completed\n📋 File is now available for use in later pipeline stages`)
      
      // Reset editing state
      setCurrentEditingFile(null)
      setIsFileBeingEdited(false)
      
      // Close the editor modal
      setShowUrdfEditor(false)
      
    } catch (error) {
      console.error('Error saving to workspace:', error)
      alert('Error saving file to workspace. Please check console for details.')
    }
  }

  const reimportFromVSCode = async () => {
    if (!currentEditingFile) {
      alert('❌ No file is currently being edited in VS Code. Please select a template or import a file first.')
      return
    }

    try {
      const workspacePath = `c:\\Users\\Ritu\\robotics-workflow\\${mechanicalWorkspacePath}`
      const filePath = `${workspacePath}\\${currentEditingFile}`
      
      // Check if we're in Electron environment with our custom APIs
      const isElectron = typeof window !== 'undefined' && (window as any).electronAPI
      
      if (isElectron) {
        try {
          console.log('Using Electron APIs to re-import file...')
          
          const readResult = (window as any).electronAPI.readFile(filePath)
          
          if (readResult.success) {
            // Update the editor with the new content
            setUrdfContent(readResult.content)
            
            // Validate the updated content
            const validation = validateUrdf(readResult.content)
            setIsUrdfValid(validation.isValid)
            setUrdfValidationErrors(validation.errors)
            
            setLastModifiedTime(new Date())
            
            alert(`✅ Successfully re-imported "${currentEditingFile}" from VS Code!\n\n📝 Content updated in the editor with ${readResult.content.length} characters.\n🔍 Validation: ${validation.isValid ? 'Valid URDF' : 'Issues found - check validation panel'}`)
            
          } else {
            alert(`❌ File not found or read error: ${readResult.error}\n\n📁 Expected location: ${filePath}\n\n🔧 Please ensure the file exists in the mechanical_design folder.`)
          }
          
        } catch (electronError) {
          console.error('Electron API error:', electronError)
          alert(`❌ Error reading file with Electron APIs.\n\n📂 Please manually select the updated "${currentEditingFile}" file.\n\nClick "Import URDF" and choose the edited file.`)
        }
      } else {
        // In browser environment, ask user to manually select the updated file
        alert(`📂 Browser environment detected.\n\nPlease manually select the updated "${currentEditingFile}" file from your mechanical_design folder.\n\nClick "Import URDF" and choose the edited file.`)
      }
      
    } catch (error) {
      console.error('Error re-importing from VS Code:', error)
      alert('❌ Error re-importing file. Please check console for details.')
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
        content: generateAIResponse(currentMessage, !!uploadedImage, selectedStep),
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
                {/* Enhanced Linear Flow Steps (Mechanical → Control → Simulation → Hardware → Export) */}
                <div className="flex items-center justify-center space-x-4">
                  {workflowSteps.map((step, index) => (
                    <React.Fragment key={step.id}>
                      <Card
                        className={`cursor-pointer transition-all duration-200 hover:shadow-lg w-64 ${
                          selectedStep === step.id ? "ring-2 ring-blue-500 shadow-lg" : ""
                        } ${step.id === "control" ? "ring-2 ring-orange-300" : ""}`}
                        onClick={() => handleCardClick(step.id)}
                        onDoubleClick={(e) => handleCardDoubleClick(step.id, e)}
                        title="Click to view details • Double-click for advanced features"
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
                                Feeds into Simulation
                              </div>
                            )}
                            {step.id === "mechanical" && (
                              <div className="text-xs text-blue-600 font-medium">
                                Click for URDF/XML Editor
                              </div>
                            )}
                            {step.features && step.features.length > 0 && (
                              <div className="text-xs text-gray-600 bg-gray-50 p-2 rounded max-w-full">
                                <div className="font-medium mb-1">Key Features:</div>
                                <div className="text-left">
                                  {step.features.slice(0, 2).map((feature, idx) => (
                                    <div key={idx} className="truncate">• {feature}</div>
                                  ))}
                                  {step.features.length > 2 && (
                                    <div className="text-blue-600">+{step.features.length - 2} more...</div>
                                  )}
                                </div>
                              </div>
                            )}
                          </div>
                        </CardContent>
                      </Card>

                      {/* Arrow between cards */}
                      {index < workflowSteps.length - 1 && (
                        <div className="flex items-center">
                          <div className="w-8 h-0.5 bg-gradient-to-r from-blue-400 to-purple-500"></div>
                          <ArrowRight className="h-6 w-6 text-purple-500 -ml-1" />
                        </div>
                      )}
                    </React.Fragment>
                  ))}
                </div>

                {/* GenAI Integration Banner */}
                <Card className="mt-6 bg-gradient-to-r from-purple-50 to-blue-50 border-purple-200">
                  <CardHeader>
                    <CardTitle className="flex items-center space-x-2">
                      <Brain className="h-5 w-5 text-purple-600" />
                      <span>GenAI Engine Integration (LLM Assistance)</span>
                    </CardTitle>
                    <CardDescription>
                      AI assistance spans across all modules with specialized robotics knowledge
                    </CardDescription>
                  </CardHeader>
                  <CardContent>
                    <div className="grid grid-cols-1 md:grid-cols-4 gap-4 text-sm">
                      <div className="bg-blue-50 p-3 rounded border">
                        <div className="font-medium text-blue-900 mb-2">Mechanical Design:</div>
                        <div>CAD assistance, URDF generation, design optimization</div>
                      </div>
                      <div className="bg-orange-50 p-3 rounded border">
                        <div className="font-medium text-orange-900 mb-2">Control System:</div>
                        <div>Code generation, controller tuning, GUI building</div>
                      </div>
                      <div className="bg-green-50 p-3 rounded border">
                        <div className="font-medium text-green-900 mb-2">Simulation:</div>
                        <div>Environment setup, task definition, report generation</div>
                      </div>
                      <div className="bg-red-50 p-3 rounded border">
                        <div className="font-medium text-red-900 mb-2">Export & Reports:</div>
                        <div>Documentation, data analysis, automated reporting</div>
                      </div>
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

                {/* Output Section */}
                <Card className="mt-8">
                  <CardHeader>
                    <CardTitle className="flex items-center space-x-2">
                      <Download className="h-5 w-5 text-blue-600" />
                      <span>Output Files & Reports</span>
                    </CardTitle>
                    <CardDescription>
                      Generated files and reports from each module in the robotics pipeline
                    </CardDescription>
                  </CardHeader>
                  <CardContent>
                    <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-4">
                      {outputCards.map((outputCard) => (
                        <Card 
                          key={outputCard.id} 
                          className="cursor-pointer transition-all duration-200 hover:shadow-lg hover:scale-105"
                          onClick={() => handleOutputCardClick(outputCard)}
                        >
                          <CardContent className="p-4">
                            <div className="flex flex-col space-y-3">
                              <div className="flex items-center justify-between">
                                <div className="flex items-center space-x-2">
                                  {outputCard.icon}
                                  <span className="font-medium text-sm">{outputCard.title}</span>
                                </div>
                                <div className="flex items-center space-x-1">
                                  {getOutputStatusIcon(outputCard.status)}
                                  <Badge variant="secondary" className={`text-xs ${getOutputStatusColor(outputCard.status)}`}>
                                    {outputCard.status.toUpperCase()}
                                  </Badge>
                                </div>
                              </div>
                              
                              <div className="text-xs text-gray-600">
                                <div className="font-medium text-blue-600 mb-1">Source: {outputCard.sourceModule}</div>
                                <div className="mb-2">{outputCard.description}</div>
                                
                                {outputCard.status === "available" && (
                                  <div className="space-y-1">
                                    <div className="flex justify-between">
                                      <span>Files:</span>
                                      <span className="text-green-600">{outputCard.files.length}</span>
                                    </div>
                                    <div className="flex justify-between">
                                      <span>Size:</span>
                                      <span className="text-green-600">{outputCard.fileSize}</span>
                                    </div>
                                    <div className="flex justify-between">
                                      <span>Generated:</span>
                                      <span className="text-gray-500">{outputCard.lastGenerated?.split(' ')[1]}</span>
                                    </div>
                                  </div>
                                )}
                                
                                {outputCard.status === "pending" && (
                                  <div className="text-gray-500 text-center py-2">
                                    Waiting for {outputCard.sourceModule} completion
                                  </div>
                                )}
                              </div>
                            </div>
                          </CardContent>
                        </Card>
                      ))}
                    </div>
                  </CardContent>
                </Card>
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
          <DialogContent className="max-w-3xl max-h-[80vh] overflow-y-auto">
            <DialogHeader>
              <DialogTitle className="flex items-center justify-between">
                <div className="flex items-center space-x-2">
                  {selectedCardDetails?.icon}
                  <span>{selectedCardDetails?.title}</span>
                </div>
                <Button
                  variant="outline"
                  size="sm"
                  onClick={() => setShowCardDetails(false)}
                  className="text-gray-500 hover:text-gray-700"
                >
                  <X className="h-4 w-4" />
                  Close
                </Button>
              </DialogTitle>
              <DialogDescription className="flex items-center justify-between">
                <span>{selectedCardDetails?.description}</span>
                <span className="text-xs text-gray-400 bg-gray-100 px-2 py-1 rounded">
                  Press ESC to close
                </span>
              </DialogDescription>
            </DialogHeader>

            <div className="space-y-6 pr-2">
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
              </div>

              {/* Features */}
              {selectedCardDetails?.features && selectedCardDetails.features.length > 0 && (
                <div>
                  <h4 className="font-medium mb-3">Key Features:</h4>
                  <div className="space-y-2 mb-3">
                    {selectedCardDetails.features.map((feature, index) => (
                      <div key={index} className="flex items-start space-x-2 text-sm">
                        <div className="w-2 h-2 bg-blue-500 rounded-full mt-2 flex-shrink-0"></div>
                        <span>{feature}</span>
                      </div>
                    ))}
                  </div>
                </div>
              )}

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

              {/* Show created/imported files for Mechanical Design */}
              {selectedCardDetails?.id === "mechanical" && mechanicalFiles.length > 0 && (
                <div className="space-y-3 pt-4 border-t">
                  <h4 className="font-medium text-sm">Created/Imported Files ({mechanicalFiles.length})</h4>
                  <div className="space-y-2">
                    {mechanicalFiles.map((file, index) => (
                      <div key={index} className="flex items-center justify-between p-2 bg-green-50 border border-green-200 rounded-lg">
                        <div className="flex items-center space-x-2">
                          <FileText className="h-4 w-4 text-green-600" />
                          <div>
                            <div className="text-sm font-medium text-gray-900">{file.name}</div>
                            <div className="text-xs text-gray-500">{file.type} • {file.createdAt}</div>
                          </div>
                        </div>
                        <div className="flex items-center space-x-2">
                          <span className="text-xs text-gray-500">{file.size}</span>
                          <Badge variant="secondary" className="bg-green-100 text-green-800">
                            ✓ Ready
                          </Badge>
                        </div>
                      </div>
                    ))}
                  </div>
                </div>
              )}

              {selectedCardDetails?.id === "mechanical" && mechanicalFiles.length === 0 && (
                <div className="space-y-3 pt-4 border-t">
                  <div className="text-center p-6 bg-gray-50 border-2 border-dashed border-gray-300 rounded-lg">
                    <FileText className="h-8 w-8 text-gray-400 mx-auto mb-2" />
                    <h4 className="text-sm font-medium text-gray-600 mb-1">No URDF/XML Files Yet</h4>
                    <p className="text-xs text-gray-500 mb-4">Create or import URDF files to get started with mechanical design.</p>
                    <Button
                      size="sm"
                      className="bg-gradient-to-r from-blue-600 to-purple-600 hover:from-blue-700 hover:to-purple-700"
                      onClick={() => {
                        setShowUrdfEditor(true)
                        setShowCardDetails(false)
                      }}
                    >
                      <Plus className="h-4 w-4 mr-1" />
                      Create URDF File
                    </Button>
                  </div>
                </div>
              )}

              {/* Action Buttons */}
              <div className="space-y-3 pt-4">
                {/* Standard action buttons */}
                <div className="flex space-x-3">
                  <Button 
                    className="flex-1"
                    onClick={() => {
                      // Close modal and show configuration options
                      setShowCardDetails(false)
                      alert(`🔧 Configuration for ${selectedCardDetails?.title}\n\nThis will open the configuration panel for this workflow step.`)
                    }}
                  >
                    <Settings className="h-4 w-4 mr-2" />
                    Configure Step
                  </Button>
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

                {/* URDF/XML Editor button - only for Mechanical Design */}
                {selectedCardDetails?.id === "mechanical" && (
                  <Button
                    className="w-full bg-gradient-to-r from-blue-600 to-purple-600 hover:from-blue-700 hover:to-purple-700"
                    onClick={() => {
                      setShowUrdfEditor(true)
                      setShowCardDetails(false)
                    }}
                  >
                    <Box className="h-4 w-4 mr-2" />
                    Open URDF/XML Editor
                  </Button>
                )}

                {/* Quick Access Navigation */}
                <div className="border-t pt-4 mt-4">
                  <Button
                    variant="outline"
                    className="w-full"
                    onClick={() => setShowCardDetails(false)}
                  >
                    <ArrowRight className="h-4 w-4 mr-2 rotate-180" />
                    Back to Workflow Overview
                  </Button>
                </div>
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

        {/* Output Details Modal */}
        <Dialog open={showOutputDetails} onOpenChange={setShowOutputDetails}>
          <DialogContent className="max-w-2xl">
            <DialogHeader>
              <DialogTitle className="flex items-center space-x-2">
                {selectedOutputCard?.icon}
                <span>{selectedOutputCard?.title}</span>
              </DialogTitle>
              <DialogDescription>
                {selectedOutputCard?.description}
              </DialogDescription>
            </DialogHeader>

            <div className="space-y-6">
              {/* Output Info */}
              <div className="grid grid-cols-2 gap-4">
                <div>
                  <span className="font-medium">Source Module:</span>
                  <div className="text-blue-600 font-medium">{selectedOutputCard?.sourceModule}</div>
                </div>
                <div>
                  <span className="font-medium">Output Type:</span>
                  <div className="text-gray-700">{selectedOutputCard?.outputType}</div>
                </div>
                <div>
                  <span className="font-medium">Status:</span>
                  <div className="flex items-center space-x-2">
                    {selectedOutputCard && getOutputStatusIcon(selectedOutputCard.status)}
                    <Badge className={`${getOutputStatusColor(selectedOutputCard?.status || "pending")}`}>
                      {selectedOutputCard?.status?.toUpperCase()}
                    </Badge>
                  </div>
                </div>
                {selectedOutputCard?.status === "available" && (
                  <div>
                    <span className="font-medium">File Size:</span>
                    <div className="text-green-600 font-medium">{selectedOutputCard?.fileSize}</div>
                  </div>
                )}
              </div>

              {/* Generated Files */}
              {selectedOutputCard?.files && selectedOutputCard.files.length > 0 && (
                <div>
                  <h4 className="font-medium mb-3">Generated Files:</h4>
                  <div className="space-y-2">
                    {selectedOutputCard.files.map((file, index) => (
                      <div key={index} className="flex items-center justify-between p-3 bg-gray-50 rounded-lg">
                        <div className="flex items-center space-x-3">
                          <FileText className="h-4 w-4 text-gray-600" />
                          <span className="text-sm font-medium">{file}</span>
                        </div>
                        {selectedOutputCard.status === "available" && (
                          <div className="flex space-x-2">
                            <Button size="sm" variant="outline" className="h-8 px-3">
                              <Eye className="h-3 w-3 mr-1" />
                              View
                            </Button>
                            <Button size="sm" variant="outline" className="h-8 px-3">
                              <Download className="h-3 w-3 mr-1" />
                              Download
                            </Button>
                          </div>
                        )}
                      </div>
                    ))}
                  </div>
                </div>
              )}

              {/* Last Generated */}
              {selectedOutputCard?.status === "available" && selectedOutputCard.lastGenerated && (
                <div className="text-sm text-gray-600 border-t pt-4">
                  <div className="flex justify-between">
                    <span>Last Generated:</span>
                    <span className="font-medium">{selectedOutputCard.lastGenerated}</span>
                  </div>
                </div>
              )}

              {/* Pending Message */}
              {selectedOutputCard?.status === "pending" && (
                <div className="text-center py-6 bg-gray-50 rounded-lg">
                  <AlertCircle className="h-8 w-8 text-gray-400 mx-auto mb-2" />
                  <div className="text-gray-600">
                    Files will be generated once {selectedOutputCard.sourceModule} module is completed.
                  </div>
                </div>
              )}

              {/* Download All Button */}
              {selectedOutputCard?.status === "available" && (
                <div className="flex justify-end">
                  <Button className="bg-blue-600 hover:bg-blue-700">
                    <Download className="h-4 w-4 mr-2" />
                    Download All Files
                  </Button>
                </div>
              )}
            </div>
          </DialogContent>
        </Dialog>

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

      {/* Output Details Modal */}
      <Dialog open={showOutputDetails} onOpenChange={setShowOutputDetails}>
        <DialogContent className="max-w-3xl">
          <DialogHeader>
            <DialogTitle className="flex items-center space-x-2">
              {selectedOutputCard?.icon}
              <span>{selectedOutputCard?.title}</span>
            </DialogTitle>
            <DialogDescription>
              {selectedOutputCard?.description} • Source: {selectedOutputCard?.sourceModule}
            </DialogDescription>
          </DialogHeader>

          <div className="space-y-6">
            {/* Status and Metadata */}
            <div className="grid grid-cols-2 gap-4">
              <div className="space-y-2">
                <div className="flex items-center justify-between">
                  <span className="font-medium">Status:</span>
                  <div className="flex items-center space-x-2">
                    {getOutputStatusIcon(selectedOutputCard?.status || "pending")}
                    <Badge className={`${getOutputStatusColor(selectedOutputCard?.status || "pending")}`}>
                      {selectedOutputCard?.status?.toUpperCase()}
                    </Badge>
                  </div>
                </div>
                <div className="flex items-center justify-between">
                  <span className="font-medium">Output Type:</span>
                  <span className="text-gray-600">{selectedOutputCard?.outputType}</span>
                </div>
              </div>
              
              {selectedOutputCard?.status === "available" && (
                <div className="space-y-2">
                  <div className="flex items-center justify-between">
                    <span className="font-medium">Total Size:</span>
                    <span className="text-green-600">{selectedOutputCard?.fileSize}</span>
                  </div>
                  <div className="flex items-center justify-between">
                    <span className="font-medium">Generated:</span>
                    <span className="text-gray-600">{selectedOutputCard?.lastGenerated}</span>
                  </div>
                </div>
              )}
            </div>

            {/* Files List */}
            <div>
              <h4 className="font-medium mb-3">Files ({selectedOutputCard?.files.length}):</h4>
              
              {selectedOutputCard?.status === "available" ? (
                <div className="space-y-2">
                  {selectedOutputCard?.files.map((file, index) => (
                    <div key={index} className="flex items-center justify-between p-3 bg-gray-50 rounded-lg">
                      <div className="flex items-center space-x-3">
                        <FileText className="h-4 w-4 text-blue-600" />
                        <span className="font-medium">{file}</span>
                      </div>
                      <div className="flex items-center space-x-2">
                        <Button variant="outline" size="sm">
                          <Eye className="h-3 w-3 mr-1" />
                          View
                        </Button>
                        <Button variant="outline" size="sm">
                          <Download className="h-3 w-3 mr-1" />
                          Download
                        </Button>
                      </div>
                    </div>
                  ))}
                  
                  {/* Bulk Actions */}
                  <div className="flex justify-end space-x-2 mt-4 pt-4 border-t">
                    <Button variant="outline">
                      <Copy className="h-4 w-4 mr-2" />
                      Copy File Paths
                    </Button>
                    <Button>
                      <Download className="h-4 w-4 mr-2" />
                      Download All Files
                    </Button>
                  </div>
                </div>
              ) : (
                <div className="text-center py-8 text-gray-500">
                  <AlertCircle className="h-12 w-12 mx-auto mb-3 text-gray-400" />
                  <p className="text-lg font-medium mb-2">Files Not Available Yet</p>
                  <p>Complete the <strong>{selectedOutputCard?.sourceModule}</strong> module to generate these files.</p>
                  
                  {selectedOutputCard?.files && (
                    <div className="mt-4 p-3 bg-gray-50 rounded-lg">
                      <p className="text-sm font-medium mb-2">Expected Files:</p>
                      <div className="space-y-1">
                        {selectedOutputCard.files.map((file, index) => (
                          <div key={index} className="text-sm text-gray-600">• {file}</div>
                        ))}
                      </div>
                    </div>
                  )}
                </div>
              )}
            </div>
          </div>
        </DialogContent>
      </Dialog>

      {/* URDF/XML Editor Modal */}
      <Dialog open={showUrdfEditor} onOpenChange={(open) => {
        if (!open) {
          // Cleanup when closing editor
          stopVSCodeWatcher()
          setCurrentEditingFile(null)
          setIsFileBeingEdited(false)
        }
        setShowUrdfEditor(open)
      }}>
        <DialogContent className="max-w-[98vw] max-h-[90vh] overflow-hidden w-[98vw] px-6 py-4">
          <DialogHeader>
            <DialogTitle className="flex items-center space-x-2">
              <FileText className="h-5 w-5 text-blue-600" />
              <span>URDF/XML Editor</span>
              <Badge variant={isUrdfValid ? "default" : "destructive"} className="ml-2">
                {isUrdfValid ? "Valid" : "Invalid"}
              </Badge>
            </DialogTitle>
            <DialogDescription>
              Create new URDF files from templates or import existing ones for editing
            </DialogDescription>
          </DialogHeader>

          <div className="flex flex-col h-[75vh]">
            {/* Toolbar */}
            <div className="flex flex-col space-y-4 p-4 border-b bg-gray-50 rounded-t">
              {/* Row 1: Template Selection */}
              <div className="flex items-center space-x-4">
                <span className="text-sm font-medium whitespace-nowrap">Template:</span>
                <Select value={selectedUrdfTemplate} onValueChange={loadUrdfTemplate}>
                  <SelectTrigger className="w-48">
                    <SelectValue placeholder="Choose template" />
                  </SelectTrigger>
                  <SelectContent>
                    {urdfTemplates.map((template) => (
                      <SelectItem key={template.id} value={template.id}>
                        <div className="flex flex-col">
                          <span>{template.name}</span>
                          <span className="text-xs text-gray-500">{template.category}</span>
                        </div>
                      </SelectItem>
                    ))}
                  </SelectContent>
                </Select>
              </div>

              {/* Row 2: VS Code Integration, File Operations and Save Actions */}
              <div className="flex items-center justify-between gap-4">
                {/* Left Section: VS Code Integration */}
                <div className="flex items-center space-x-4">
                  <div className="flex items-center space-x-2 p-2 border rounded bg-white">
                    <input
                      type="checkbox"
                      id="vscode-checkbox"
                      checked={openWithVSCode}
                      onChange={(e) => setOpenWithVSCode(e.target.checked)}
                      className="w-4 h-4 text-blue-600 bg-gray-100 border-gray-300 rounded focus:ring-blue-500"
                    />
                    <label htmlFor="vscode-checkbox" className="text-sm font-medium text-gray-700 cursor-pointer whitespace-nowrap">
                      Open with VS Code
                    </label>
                  </div>
                </div>

                {/* Right Section: File Operations */}
                <div className="flex items-center space-x-2">
                  <input
                    type="file"
                    accept=".urdf,.xml"
                    onChange={handleFileImport}
                    className="hidden"
                    id="urdf-file-input"
                  />
                  <Button
                    variant="outline"
                    size="sm"
                    onClick={() => document.getElementById('urdf-file-input')?.click()}
                    className="whitespace-nowrap bg-blue-50 hover:bg-blue-100 border-blue-200"
                  >
                    <Upload className="h-4 w-4 mr-2" />
                    Import URDF
                  </Button>
                  
                  <Button
                    variant="outline"
                    size="sm"
                    onClick={downloadUrdf}
                    disabled={!urdfContent}
                    className="whitespace-nowrap bg-green-50 hover:bg-green-100 border-green-200"
                  >
                    <Download className="h-4 w-4 mr-2" />
                    Export
                  </Button>
                </div>
              </div>

              {/* Row 3: File Name */}
              <div className="flex items-center justify-between">
                <div className="flex items-center space-x-2">
                  <span className="text-sm font-medium">File Name:</span>
                  <input
                    type="text"
                    value={urdfFileName}
                    onChange={(e) => setUrdfFileName(e.target.value)}
                    className="px-2 py-1 text-sm border rounded w-48"
                    placeholder="Enter filename..."
                  />
                </div>
                
                <div className="flex items-center space-x-2 text-sm text-gray-600">
                  <span>Ready for editing</span>
                </div>
              </div>
            </div>

            {/* Editor Content Area */}
            <div className="flex flex-1 overflow-hidden">
              {/* Editor Panel */}
              <div className="flex-1 flex flex-col">
                <div className="flex items-center justify-between p-2 bg-gray-100 border-b">
                  <h3 className="text-sm font-medium">URDF Editor</h3>
                  <div className="flex items-center space-x-2">
                    <Button
                      variant="outline"
                      size="sm"
                      onClick={() => setShowUrdfPreview(!showUrdfPreview)}
                    >
                      <Eye className="h-4 w-4 mr-1" />
                      {showUrdfPreview ? 'Hide' : 'Show'} Preview
                    </Button>
                  </div>
                </div>
                
                <Textarea
                  value={urdfContent}
                  onChange={(e) => handleUrdfContentChange(e.target.value)}
                  className="flex-1 font-mono text-sm resize-none rounded-none border-0"
                  placeholder="Enter your URDF content here or select a template above...

Example URDF structure:
<?xml version='1.0'?>
<robot name='my_robot'>
  <link name='base_link'>
    <visual>
      <geometry>
        <box size='1 1 1'/>
      </geometry>
    </visual>
  </link>
</robot>"
                  style={{ minHeight: '400px' }}
                />
              </div>

              {/* Preview/Validation Panel */}
              {showUrdfPreview && (
                <div className="w-1/3 border-l">
                  <div className="p-2 bg-gray-100 border-b">
                    <h3 className="text-sm font-medium">Validation & Info</h3>
                  </div>
                  <div className="p-4 overflow-y-auto" style={{ maxHeight: '400px' }}>
                    {/* Validation Status */}
                    <div className="mb-4">
                      <h4 className="text-sm font-medium mb-2">Validation Status</h4>
                      {isUrdfValid ? (
                        <div className="flex items-center space-x-2 text-green-600">
                          <CheckCircle className="h-4 w-4" />
                          <span className="text-sm">URDF is valid</span>
                        </div>
                      ) : (
                        <div className="space-y-2">
                          <div className="flex items-center space-x-2 text-red-600">
                            <AlertCircle className="h-4 w-4" />
                            <span className="text-sm">URDF has errors</span>
                          </div>
                          <div className="space-y-1">
                            {urdfValidationErrors.map((error, index) => (
                              <div key={index} className="text-xs text-red-600 bg-red-50 p-2 rounded">
                                {error}
                              </div>
                            ))}
                          </div>
                        </div>
                      )}
                    </div>

                    {/* File Statistics */}
                    <div className="mb-4">
                      <h4 className="text-sm font-medium mb-2">Statistics</h4>
                      <div className="space-y-1 text-xs text-gray-600">
                        <div>Lines: {urdfContent.split('\n').length}</div>
                        <div>Characters: {urdfContent.length}</div>
                        <div>Links: {(urdfContent.match(/<link/g) || []).length}</div>
                        <div>Joints: {(urdfContent.match(/<joint/g) || []).length}</div>
                      </div>
                    </div>

                    {/* Quick Help */}
                    <div>
                      <h4 className="text-sm font-medium mb-2">Quick Help</h4>
                      <div className="space-y-2 text-xs text-gray-600">
                        <div>• Every robot needs a name attribute</div>
                        <div>• Links define robot parts</div>
                        <div>• Joints connect links</div>
                        <div>• Visual elements define appearance</div>
                        <div>• Collision elements define physics</div>
                        <div>• Inertial elements define mass properties</div>
                      </div>
                    </div>
                  </div>
                </div>
              )}
            </div>

            {/* Action Buttons */}
            <div className="flex items-center justify-between p-4 border-t bg-gray-50">
              {/* Left Section: Editor Actions */}
              <div className="flex items-center space-x-2">
                <Button
                  variant="outline"
                  size="sm"
                  onClick={() => {
                    setUrdfContent("")
                    setSelectedUrdfTemplate("blank")
                    setUrdfFileName("robot_model.urdf")
                    setIsUrdfValid(true)
                    setUrdfValidationErrors([])
                  }}
                >
                  Clear Editor
                </Button>
                <Button
                  variant="outline"
                  size="sm"
                  onClick={() => loadUrdfTemplate("blank")}
                >
                  Reset to Blank
                </Button>
              </div>
              
              {/* Right Section: File Actions */}
              <div className="flex items-center space-x-2">
                <Button
                  variant="outline"
                  size="sm"
                  onClick={() => setShowUrdfEditor(false)}
                  className="bg-transparent"
                >
                  Close
                </Button>
                <Button
                  onClick={saveToMechanicalWorkspace}
                  size="sm"
                  disabled={!isUrdfValid || !urdfContent}
                  className="bg-gradient-to-r from-green-600 to-blue-600 hover:from-green-700 hover:to-blue-700"
                >
                  <Box className="h-4 w-4 mr-2" />
                  Save to Workspace
                </Button>
              </div>
            </div>
          </div>
        </DialogContent>
      </Dialog>
    </div>
  )
}