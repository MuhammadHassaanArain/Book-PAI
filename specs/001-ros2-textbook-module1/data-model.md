# Data Model: Module 1 — The Robotic Nervous System (ROS 2)

## Educational Content Entities

### TopicFile
- **name**: String (e.g., "1.1-introduction.md")
- **title**: String (e.g., "Introduction to ROS 2")
- **wordCount**: Integer (800-1200)
- **contentSections**: Array of ContentSection
- **codeExamples**: Array of CodeExample
- **diagramDescriptions**: Array of DiagramDescription
- **references**: Array of Reference
- **learningObjectives**: Array of String
- **summaryPoints**: Array of String
- **checklistItems**: Array of String

### ContentSection
- **title**: String
- **body**: String (textbook-quality prose)
- **humanoidContext**: String (relevance to humanoid robotics)
- **keyConcepts**: Array of String

### CodeExample
- **language**: String (e.g., "python", "bash", "xml")
- **code**: String (executable code snippet)
- **explanation**: String (detailed explanation of code)
- **expectedOutput**: String (what the code should produce)

### DiagramDescription
- **title**: String
- **purpose**: String (what the diagram illustrates)
- **components**: Array of String (elements in the diagram)
- **textualDescription**: String (detailed text description)

### Reference
- **type**: String (e.g., "academic", "documentation", "whitepaper")
- **citation**: String (APA 7th edition format)
- **url**: String (if applicable)
- **relevance**: String (why this source is relevant)

## ROS 2 System Entities

### ROS2Node
- **nodeName**: String
- **nodeDescription**: String
- **publishers**: Array of Publisher
- **subscribers**: Array of Subscriber
- **services**: Array of Service
- **actions**: Array of Action
- **parameters**: Array of Parameter

### Publisher
- **topicName**: String
- **messageType**: String (e.g., "std_msgs/msg/String")
- **qosProfile**: QoSProfile
- **frequency**: Float (Hz, if applicable)

### Subscriber
- **topicName**: String
- **messageType**: String
- **qosProfile**: QoSProfile
- **callbackFunction**: String (name of callback function)

### Service
- **serviceName**: String
- **serviceType**: String (e.g., "std_srvs/srv/SetBool")
- **isServer**: Boolean (true for server, false for client)

### Action
- **actionName**: String
- **actionType**: String (e.g., "example_interfaces/action/Fibonacci")
- **isServer**: Boolean (true for server, false for client)

### Parameter
- **parameterName**: String
- **parameterType**: String (e.g., "string", "int", "double", "bool")
- **defaultValue**: Any
- **description**: String

### QoSProfile
- **reliability**: String ("reliable" or "best_effort")
- **durability**: String ("volatile" or "transient_local")
- **history**: String ("keep_last" or "keep_all")
- **depth**: Integer (for keep_last history)
- **deadline**: Duration
- **liveliness**: String ("automatic" or "manual_by_topic")

## URDF Entities

### URDFRobot
- **robotName**: String
- **links**: Array of URDFLink
- **joints**: Array of URDFJoint
- **materials**: Array of URDFMaterial
- **transmissions**: Array of URDFTransmission
- **gazeboPlugins**: Array of GazeboPlugin

### URDFLink
- **linkName**: String
- **visual**: URDFVisual
- **collision**: URDFVisual
- **inertial**: URDFInertial
- **description**: String

### URDFJoint
- **jointName**: String
- **jointType**: String ("revolute", "continuous", "prismatic", "fixed", etc.)
- **parentLink**: String
- **childLink**: String
- **origin**: URDFOrigin
- **axis**: URDFVector (for revolute/prismatic joints)
- **limits**: URDFJointLimits (for revolute/prismatic joints)

### URDFOrigin
- **xyz**: Array of Float (3 elements: x, y, z)
- **rpy**: Array of Float (3 elements: roll, pitch, yaw)

### URDFVector
- **x**: Float
- **y**: Float
- **z**: Float

### URDFInertial
- **mass**: Float
- **inertia**: URDFInertiaMatrix
- **origin**: URDFOrigin

### URDFInertiaMatrix
- **ixx**: Float
- **ixy**: Float
- **ixz**: Float
- **iyy**: Float
- **iyz**: Float
- **izz**: Float

### URDFVisual
- **geometry**: URDFGeometry
- **material**: URDFMaterial
- **origin**: URDFOrigin

### URDFGeometry
- **type**: String ("box", "cylinder", "sphere", "mesh")
- **dimensions**: Array of Float (varies by type)
- **filename**: String (for mesh type)

### URDFMaterial
- **name**: String
- **color**: URDFColor
- **texture**: String (optional)

### URDFColor
- **r**: Float (0.0-1.0)
- **g**: Float (0.0-1.0)
- **b**: Float (0.0-1.0)
- **a**: Float (0.0-1.0)

### URDFJointLimits
- **effort**: Float
- **velocity**: Float
- **lower**: Float (for revolute joints)
- **upper**: Float (for revolute joints)

### URDFTransmission
- **name**: String
- **type**: String (e.g., "transmission_interface/SimpleTransmission")
- **joint**: URDFJointTransmission
- **actuator**: URDFActuator

### URDFJointTransmission
- **name**: String
- **hardwareInterface**: String (e.g., "hardware_interface/EffortJointInterface")

### URDFActuator
- **name**: String
- **mechanicalReduction**: Float

## Launch System Entities

### LaunchFile
- **fileName**: String
- **description**: String
- **nodes**: Array of LaunchNode
- **parameters**: Array of LaunchParameter
- **remappings**: Array of LaunchRemapping
- **conditions**: Array of LaunchCondition

### LaunchNode
- **packageName**: String
- **executableName**: String
- **nodeName**: String
- **parameters**: Array of LaunchParameter
- **remappings**: Array of LaunchRemapping
- **arguments**: Array of String

### LaunchParameter
- **name**: String
- **value**: Any
- **sourceFile**: String (optional, for YAML parameter files)

### LaunchRemapping
- **fromTopic**: String
- **toTopic**: String

### LaunchCondition
- **conditionType**: String (e.g., "IfCondition", "UnlessCondition")
- **value**: Boolean or String expression

## Assessment Entities

### LabGuide
- **title**: String
- **objective**: String
- **prerequisites**: Array of String
- **steps**: Array of LabStep
- **expectedOutput**: String
- **verificationChecklist**: Array of String
- **troubleshootingTips**: Array of String

### LabStep
- **stepNumber**: Integer
- **description**: String
- **command**: String (if applicable)
- **expectedResult**: String

### MiniProject
- **title**: String
- **objective**: String
- **componentsRequired**: Array of String
- **implementationSteps**: Array of ImplementationStep
- **successCriteria**: Array of String
- **extensionOpportunities**: Array of String

### ImplementationStep
- **stepNumber**: Integer
- **description**: String
- **codeOrConfiguration**: String
- **verificationMethod**: String