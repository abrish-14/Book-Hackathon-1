# Data Model: AI-Robot Brain Documentation Module

## Documentation Module Structure

### Entity: Documentation Module
- **Name**: AI-Robot Brain
- **Description**: A structured section of the Docusaurus site dedicated to NVIDIA Isaac technologies
- **Fields**:
  - title: "AI-Robot Brain (NVIDIA Isaac™)"
  - description: "Documentation for NVIDIA Isaac Sim, Isaac ROS, and Nav2 technologies"
  - navigation_order: 3 (Module 3)
  - category: "Robotics Framework"
- **Relationships**: Contains 3 main chapters

### Entity: Chapter
- **Name**: Isaac Technology Chapter
- **Description**: Individual documentation chapters covering specific Isaac technologies
- **Fields**:
  - title: Chapter title (e.g., "NVIDIA Isaac Sim for photorealistic simulation")
  - slug: URL-friendly identifier
  - content_type: "tutorial", "reference", or "conceptual"
  - prerequisites: List of required knowledge
  - examples: List of practical examples included
  - navigation_order: Order within the module
- **Relationships**: Belongs to Documentation Module
- **Validation**: Must have unique slug within module, must include practical examples

### Entity: Documentation Page
- **Name**: Individual Documentation Page
- **Description**: Single pages within each chapter containing specific information
- **Fields**:
  - title: Page title
  - slug: URL-friendly identifier
  - content: Markdown content
  - metadata: Frontmatter with description, tags, etc.
  - parent_chapter: Reference to parent chapter
- **Relationships**: Belongs to Chapter

## Navigation Structure

### Entity: Navigation Menu
- **Name**: Module Navigation
- **Description**: Navigation menu for the AI-Robot Brain module
- **Fields**:
  - items: List of navigation items
  - active_state: Current page highlighting
- **Relationships**: Contains references to Documentation Module and its Chapters

## Content Structure

### Entity: Content Block
- **Name**: Content Block
- **Description**: Reusable content components within documentation
- **Fields**:
  - type: "code_example", "tutorial_step", "concept_explanation", "best_practices"
  - content: The actual content
  - tags: Associated technology tags
- **Relationships**: Referenced by Documentation Page

## State Transitions
- Draft → Review: Content is ready for review
- Review → Published: Content approved and published
- Published → Updated: Content updated with new information