# Documentation Embedding Schema

## Purpose
This document defines the data structure for representing documentation content that will be used for RAG (Retrieval-Augmented Generation) embeddings.

## Embedding Data Structure

### Document Chunk
```json
{
  "id": "unique-identifier-for-chunk",
  "documentId": "identifier-for-original-document",
  "title": "Title of the document section",
  "content": "The actual text content for embedding",
  "metadata": {
    "sourceFile": "path/to/original/file.md",
    "url": "relative-url-to-document",
    "section": "section-name",
    "level": 2,
    "tags": ["tag1", "tag2"],
    "version": "1.0.0",
    "lastModified": "2025-12-31T00:00:00Z"
  },
  "embedding": [0.1, 0.3, 0.5, ...], // Array of float values
  "chunkIndex": 0,
  "totalChunks": 1
}
```

### Content Processing Pipeline
1. **Extraction**: Extract content from Markdown files in docs/
2. **Segmentation**: Split content into semantically meaningful chunks
3. **Enrichment**: Add metadata to each chunk
4. **Embedding**: Generate vector embeddings for each chunk
5. **Storage**: Store in vector database with metadata

### Chunking Strategy
- **Maximum Length**: 1000 tokens per chunk
- **Overlap**: 100 tokens overlap between adjacent chunks
- **Boundaries**: Respect section headers, paragraphs, and sentences
- **Context Preservation**: Maintain document hierarchy in metadata

### Metadata Schema
```json
{
  "sourceFile": "string",           // Original file path
  "url": "string",                  // Docusaurus URL for the content
  "title": "string",               // Document title
  "section": "string",             // Section name (e.g., "module-1/chapter-1")
  "level": "integer",              // Heading level in document hierarchy
  "tags": ["string"],              // Associated tags/keywords
  "version": "string",             // Document version
  "lastModified": "string",        // ISO date string
  "wordCount": "integer",          // Number of words in chunk
  "charCount": "integer",          // Number of characters in chunk
  "parentId": "string",            // ID of parent section (if applicable)
  "breadcrumbs": ["string"]        // Path from root to this section
}
```

### Example Processing
For a document `docs/module-1/chapter-1.md`:

**Original Content:**
```markdown
# Introduction to AI/Spec-Driven Development

AI/Spec-Driven Development is an approach that combines artificial intelligence...

## Core Principles

### Principle 1: Specification First
The specification comes first before any implementation...
```

**Would be processed into chunks like:**

Chunk 1:
```json
{
  "id": "mod1-ch1-intro-0",
  "documentId": "mod1-ch1",
  "title": "Introduction to AI/Spec-Driven Development",
  "content": "AI/Spec-Driven Development is an approach that combines artificial intelligence...",
  "metadata": {
    "sourceFile": "docs/module-1/chapter-1.md",
    "url": "/docs/module-1/chapter-1",
    "section": "module-1/chapter-1",
    "level": 1,
    "tags": ["ai", "spec-driven", "introduction"],
    "version": "1.0.0",
    "lastModified": "2025-12-31T00:00:00Z"
  },
  "chunkIndex": 0,
  "totalChunks": 3
}
```

Chunk 2:
```json
{
  "id": "mod1-ch1-principles-0",
  "documentId": "mod1-ch1",
  "title": "Core Principles",
  "content": "## Core Principles\n\n### Principle 1: Specification First\nThe specification comes first before any implementation...",
  "metadata": {
    "sourceFile": "docs/module-1/chapter-1.md",
    "url": "/docs/module-1/chapter-1#core-principles",
    "section": "module-1/chapter-1/core-principles",
    "level": 2,
    "tags": ["principles", "specification"],
    "version": "1.0.0",
    "lastModified": "2025-12-31T00:00:00Z"
  },
  "chunkIndex": 1,
  "totalChunks": 3
}
```

## Processing Pipeline Implementation

### 1. Content Extractor
- Reads Markdown files from docs/ directory
- Preserves document structure and metadata
- Extracts text content while maintaining semantic boundaries

### 2. Chunker
- Splits content based on semantic boundaries
- Respects document hierarchy (headings, paragraphs)
- Maintains context through overlapping chunks if needed

### 3. Metadata Enricher
- Adds relevant metadata to each chunk
- Links to original document location
- Preserves document hierarchy information

### 4. Embedding Generator
- Converts text content to vector embeddings
- Uses consistent embedding model
- Stores embedding vectors with associated metadata

## Integration Points

### With Docusaurus
- Hooks into build process to extract content
- Maintains URL mappings for result linking
- Updates embeddings when content changes

### With Vector Database
- Schema compatible with popular vector databases
- Efficient indexing strategy
- Query optimization through metadata filtering