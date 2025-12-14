# Module 3 Content Template

## Document Structure Template

```markdown
---
sidebar_position: X
---

# Chapter X.X Chapter Title

## Learning Objectives

By the end of this chapter, students will be able to:
1. [Objective 1]
2. [Objective 2]
3. [Objective 3]

## Prerequisites

Before starting this chapter, students should:
- [Prerequisite 1]
- [Prerequisite 2]
- [Prerequisite 3]

## Introduction

[Provide context and motivation for the chapter content]

## Main Content

### Section 1: [Title]

[Content for the first section]

#### Subsection 1.1: [Title]
[Detailed content for subsection]

### Section 2: [Title]
[Content for the second section]

## Practical Implementation

### Step-by-Step Guide
1. [Step 1 with detailed instructions]
2. [Step 2 with detailed instructions]
3. [Step 3 with detailed instructions]

### Code Examples
```python
# Example code with explanations
def example_function():
    """
    Example function with docstring
    """
    pass
```

## Troubleshooting

### Common Issues
- **Issue 1**: [Description and solution]
- **Issue 2**: [Description and solution]
- **Issue 3**: [Description and solution]

## Summary

[Summary of key concepts covered in the chapter]

## Knowledge Check

1. [Question 1]
2. [Question 2]
3. [Question 3]

## References

[Include APA 7th Edition references here]
```

## Lab Exercise Template

```markdown
---
sidebar_position: X
---

# Lab X: Lab Title

## Objective

[Clear statement of what students will accomplish]

## Prerequisites

- [Prerequisite 1]
- [Prerequisite 2]
- [Prerequisite 3]

## Estimated Time

[X-Y hours]

## Equipment/Software Required

- [Item 1]
- [Item 2]
- [Item 3]

## Procedure

### Setup
1. [Setup step 1]
2. [Setup step 2]

### Main Tasks
#### Task 1: [Title]
1. [Step 1]
2. [Step 2]
3. [Step 3]

#### Task 2: [Title]
1. [Step 1]
2. [Step 2]

### Validation
[How to verify successful completion]

## Troubleshooting

### Common Problems
- [Problem and solution 1]
- [Problem and solution 2]

## Assessment

### Knowledge Check Questions
1. [Question 1]
2. [Question 2]

### Practical Assessment
- [Assessment criteria 1]
- [Assessment criteria 2]

### Deliverables
1. [Deliverable 1]
2. [Deliverable 2]

## Extensions (Optional)

[Advanced topics or additional challenges]

## References

[Include APA 7th Edition references here]
```

## Code Block Template

### Python Code Example
```python
"""
Module: [Module Name]
Description: [Brief description]
Author: [Author name]
Date: [Date]
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


class ExampleNode(Node):
    """
    Example ROS 2 node demonstrating best practices
    """
    def __init__(self):
        super().__init__('example_node')

        # Create subscriptions
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Create publishers
        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.get_logger().info('Example node initialized')

    def image_callback(self, msg):
        """
        Callback function for image subscription

        Args:
            msg: Image message from camera
        """
        # Process image data
        self.get_logger().info(f'Received image with dimensions: {msg.width}x{msg.height}')

    def destroy_node(self):
        """
        Cleanup function called when node is destroyed
        """
        super().destroy_node()


def main(args=None):
    """
    Main function to run the example node
    """
    rclpy.init(args=args)
    node = ExampleNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Configuration File Example
```yaml
# Configuration file for [System/Component]
# Description: [Brief description of what this configures]
# Version: [Version number]
# Last Updated: [Date]

namespace: "robot_name"  # Robot namespace for multi-robot systems
use_sim_time: true       # Use simulation time in Gazebo/Isaac Sim

# Component-specific parameters
component_name:
  ros__parameters:
    # Parameter configuration
    update_frequency: 10.0      # Hz - update rate
    timeout: 5.0               # seconds - timeout value
    enable_feature: true       # boolean - enable/disable feature

    # Nested parameter groups
    sub_component:
      param1: value1
      param2: value2

    # Array/list parameters
    array_param: [1, 2, 3, 4]

    # Dictionary/object parameters
    dict_param:
      key1: value1
      key2: value2
```

## Documentation Standards

### Headings Hierarchy
```
# Chapter Title (H1 - One per document)
## Section Title (H2 - Major sections)
### Subsection Title (H3 - Subsections)
#### Sub-subsection Title (H4 - Detailed subsections)
```

### Formatting Standards
- **Bold text**: Use for emphasis and key terms
- *Italic text*: Use for book titles and foreign terms
- `Code text`: Use for file names, commands, and code snippets
- [Links]: Use descriptive link text

### Image Inclusion
```markdown
![Descriptive Alt Text](/assets/images/module-3/image-name.png)

*Figure X: Description of the image*
```

### Table Format
```markdown
| Column 1 | Column 2 | Column 3 |
|----------|----------|----------|
| Data 1   | Data 2   | Data 3   |
| Data 4   | Data 5   | Data 6   |
```

## Quality Assurance Checklist

Before publishing any content, verify:

### Content Structure
- [ ] Clear learning objectives
- [ ] Appropriate prerequisites listed
- [ ] Logical flow of information
- [ ] Summary section included
- [ ] Knowledge check questions

### Technical Accuracy
- [ ] All code examples tested and functional
- [ ] Commands verified to work as described
- [ ] File paths are accurate
- [ ] Dependencies properly documented
- [ ] Troubleshooting section complete

### Formatting Consistency
- [ ] Consistent heading hierarchy
- [ ] Proper code block formatting
- [ ] Correct use of emphasis and styling
- [ ] All images properly referenced
- [ ] Tables formatted correctly

### References and Citations
- [ ] All in-text citations present
- [ ] Reference list complete and properly formatted
- [ ] APA 7th Edition compliance verified
- [ ] At least 60% peer-reviewed sources (where applicable)

## Common Patterns

### Warning/Note Boxes
```markdown
> **Note**: Additional information that supplements the main content.

> **Warning**: Critical information about potential problems.

> **Tip**: Helpful hints or best practices.
```

### Command Examples
```bash
# Single line command
command --option value

# Multi-line command (use for complex commands)
command \
  --option1 value1 \
  --option2 value2 \
  --option3 value3
```

### File Structure Documentation
```
project_directory/
├── src/
│   ├── main.py
│   └── utils/
│       └── helper.py
├── config/
│   └── parameters.yaml
└── README.md
```

This template ensures consistent formatting and structure across all Module 3 content.
```