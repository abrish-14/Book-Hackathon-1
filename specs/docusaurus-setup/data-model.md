# Data Model: Docusaurus Documentation Structure

## Documentation Entity

**Fields:**
- id: string (unique identifier for the document)
- title: string (display title of the document)
- sidebar_label: string (label to show in sidebar)
- sidebar_position: number (position in sidebar hierarchy)
- custom_edit_url: string (optional, custom URL for edit button)
- description: string (meta description for SEO)

**Relationships:**
- Parent: Category (optional, for hierarchical organization)
- Children: Array<Document> (sub-documents in the same module)

## Module Entity

**Fields:**
- id: string (unique identifier for the module)
- title: string (display title of the module)
- position: number (position in main navigation)
- description: string (brief description of the module)

**Relationships:**
- Documents: Array<Document> (chapters within the module)

## Chapter Entity

**Fields:**
- id: string (unique identifier for the chapter)
- title: string (display title of the chapter)
- module_id: string (reference to parent module)
- position: number (position within the module)
- content_path: string (relative path to markdown file)

**Relationships:**
- Module: Module (parent module)
- Previous: Chapter (optional, previous chapter in sequence)
- Next: Chapter (optional, next chapter in sequence)

## Sidebar Configuration

**Structure:**
- Type: "category" or "doc"
- Label: string (display name)
- Items: Array<SidebarItem>
- Collapsible: boolean (whether the category can be collapsed)
- Collapsed: boolean (initial collapsed state)

## Validation Rules

1. Each document must have a unique ID within its module
2. Each module must have at least one document
3. Document titles must be non-empty
4. Sidebar positions must be positive integers
5. Document paths must correspond to actual markdown files

## State Transitions

1. **Draft** → **Review** → **Published** (content workflow)
2. **Unlisted** → **Public** (visibility workflow)

## Navigation Hierarchy

```
Home
├── Introduction
├── Module 1
│   ├── Chapter 1
│   ├── Chapter 2
│   └── Chapter 3
├── Module 2
│   ├── Chapter 1
│   ├── Chapter 2
│   └── Chapter 3
└── ...
```