// indexer.ts
import fs from 'fs';
import path from 'path';
// Note: In a real implementation, we would use appropriate libraries like 'glob' for file discovery
// For this placeholder, we'll simulate the functionality

interface DocumentChunk {
  id: string;
  documentId: string;
  title: string;
  content: string;
  metadata: {
    sourceFile: string;
    url: string;
    section: string;
    level: number;
    tags: string[];
    version: string;
    lastModified: string;
  };
  chunkIndex: number;
  totalChunks: number;
}

class DocumentationIndexer {
  private docsPath: string;
  private outputPath: string;

  constructor(docsPath: string = './docs', outputPath: string = './indexed-content') {
    this.docsPath = docsPath;
    this.outputPath = outputPath;
  }

  async indexDocuments(): Promise<DocumentChunk[]> {
    console.log('Starting documentation indexing process...');

    // Find all markdown files in docs directory
    // In a real implementation, this would use a glob library to find files
    const markdownFiles = this.findMarkdownFiles(this.docsPath);
    console.log(`Found ${markdownFiles.length} markdown files to process`);

    const allChunks: DocumentChunk[] = [];

    for (const file of markdownFiles) {
      console.log(`Processing file: ${file}`);
      const chunks = await this.processFile(file);
      allChunks.push(...chunks);
    }

    console.log(`Indexing complete. Processed ${allChunks.length} content chunks`);

    // In a real implementation, this would save to a vector database
    await this.saveIndex(allChunks);

    return allChunks;
  }

  private findMarkdownFiles(docsPath: string): string[] {
    // This is a simplified implementation
    // In a real implementation, we would recursively walk the directory
    // and find all .md files

    // For this placeholder, we'll return a mock list
    // In actual implementation, we'd use fs.readdirSync, path.join, etc.
    return [
      path.join(docsPath, 'intro.md'),
      path.join(docsPath, 'module-1', 'chapter-1.md'),
      path.join(docsPath, 'module-1', 'chapter-2.md'),
      path.join(docsPath, 'module-1', 'chapter-3.md')
    ].filter(file => fs.existsSync(file));
  }

  private async processFile(filePath: string): Promise<DocumentChunk[]> {
    const content = fs.readFileSync(filePath, 'utf-8');
    const relativePath = path.relative(this.docsPath, filePath);
    const documentId = this.generateDocumentId(relativePath);

    // Extract metadata from file
    const metadata = this.extractMetadata(filePath, relativePath, documentId, content);

    // Parse content into semantic chunks
    const chunks = this.parseContentIntoChunks(content, documentId, metadata);

    return chunks;
  }

  private extractMetadata(filePath: string, relativePath: string, documentId: string, content: string) {
    const stats = fs.statSync(filePath);
    const url = this.convertPathToUrl(relativePath);
    const section = this.extractSectionFromPath(relativePath);

    return {
      sourceFile: relativePath,
      url,
      section,
      level: this.extractHeadingLevel(content),
      tags: this.extractTagsFromContent(content),
      version: '1.0.0', // Would come from version control or config
      lastModified: stats.mtime.toISOString()
    };
  }

  private extractHeadingLevel(content: string): number {
    // Extract the highest level heading from the content
    const headingMatch = content.match(/^(#{1,6})\s+.*/m);
    if (headingMatch) {
      return headingMatch[1].length;
    }
    return 0;
  }

  private parseContentIntoChunks(content: string, documentId: string, metadata: any): DocumentChunk[] {
    // This is a simplified chunking algorithm
    // In a real implementation, would use more sophisticated text splitting
    const chunks: DocumentChunk[] = [];
    const maxChunkSize = 1000; // Approximate token count
    const overlap = 100;

    // Simple approach: split by headers, then by paragraphs if too large
    const sections = this.splitByHeaders(content);

    for (let i = 0; i < sections.length; i++) {
      const section = sections[i];
      const sectionChunks = this.splitLargeSection(section, maxChunkSize, overlap);

      for (let j = 0; j < sectionChunks.length; j++) {
        chunks.push({
          id: `${documentId}-chunk-${i}-${j}`,
          documentId,
          title: this.extractTitle(section),
          content: sectionChunks[j],
          metadata: { ...metadata },
          chunkIndex: j,
          totalChunks: sectionChunks.length
        });
      }
    }

    return chunks;
  }

  private splitByHeaders(content: string): string[] {
    // Split content by markdown headers
    const headerRegex = /^(#{1,6})\s+(.*)$/gm;
    const parts: string[] = [];
    let lastIndex = 0;
    let match;

    while ((match = headerRegex.exec(content)) !== null) {
      // Add content before this header
      if (match.index > lastIndex) {
        parts.push(content.substring(lastIndex, match.index));
      }

      // Add the header and its following content
      const headerEnd = content.indexOf('\n', match.index + match[0].length);
      const nextHeaderStart = content.indexOf('\n#', headerEnd);

      if (nextHeaderStart !== -1) {
        parts.push(content.substring(match.index, nextHeaderStart));
        lastIndex = nextHeaderStart;
      } else {
        parts.push(content.substring(match.index));
        break;
      }
    }

    // Add any remaining content
    if (lastIndex < content.length) {
      parts.push(content.substring(lastIndex));
    }

    // Filter out empty parts
    return parts.filter(part => part.trim().length > 0);
  }

  private splitLargeSection(section: string, maxChunkSize: number, overlap: number): string[] {
    const chunks: string[] = [];
    const words = section.split(/\s+/);

    if (words.length <= maxChunkSize) {
      chunks.push(section);
      return chunks;
    }

    // Split into chunks of maxChunkSize with overlap
    for (let i = 0; i < words.length; i += maxChunkSize - overlap) {
      const chunkWords = words.slice(i, i + maxChunkSize);
      chunks.push(chunkWords.join(' '));
    }

    return chunks;
  }

  private extractTitle(content: string): string {
    // Extract title from the first heading in the content
    const titleMatch = content.match(/^(#{1,6})\s+(.*)$/m);
    return titleMatch ? titleMatch[2].trim() : 'Untitled Section';
  }

  private extractSectionFromPath(relativePath: string): string {
    // Convert file path to section identifier
    return relativePath
      .replace(/\.md$/, '')
      .replace(/\\/g, '/')
      .replace(/^\/+|\/+$/g, '');
  }

  private convertPathToUrl(relativePath: string): string {
    // Convert file path to Docusaurus URL
    return `/docs/${relativePath.replace(/\.md$/, '')}`;
  }

  private generateDocumentId(relativePath: string): string {
    // Generate a unique ID for the document
    return relativePath
      .replace(/\\/g, '/')
      .replace(/[^a-zA-Z0-9-_]/g, '-')
      .replace(/-+/g, '-')
      .replace(/^-+|-+$/g, '');
  }

  private extractTagsFromContent(content: string): string[] {
    // Extract tags from frontmatter or content
    const tags: string[] = [];

    // Look for tags in frontmatter
    const frontmatterMatch = content.match(/---\n([\s\S]*?)\n---/);
    if (frontmatterMatch) {
      const frontmatter = frontmatterMatch[1];
      const tagsMatch = frontmatter.match(/tags:\s*\[(.*?)\]/);
      if (tagsMatch) {
        tags.push(...tagsMatch[1].split(',').map(tag => tag.trim().replace(/['"]/g, '')));
      }
    }

    return tags;
  }

  private async saveIndex(chunks: DocumentChunk[]): Promise<void> {
    // In a real implementation, this would save to a vector database
    // For now, we'll just save to a JSON file as a placeholder

    if (!fs.existsSync(this.outputPath)) {
      fs.mkdirSync(this.outputPath, { recursive: true });
    }

    const indexPath = path.join(this.outputPath, 'index.json');
    fs.writeFileSync(indexPath, JSON.stringify(chunks, null, 2));

    console.log(`Index saved to ${indexPath}`);
  }

  async runIndexing(): Promise<void> {
    try {
      await this.indexDocuments();
      console.log('Documentation indexing completed successfully');
    } catch (error) {
      console.error('Error during documentation indexing:', error);
      throw error;
    }
  }
}

// Content Watcher for development
class ContentWatcher {
  private indexer: DocumentationIndexer;
  private watcher: any | null = null; // Using any since we're not implementing actual watcher

  constructor(indexer: DocumentationIndexer) {
    this.indexer = indexer;
  }

  startWatching(docsPath: string): void {
    console.log(`Starting to watch ${docsPath} for changes...`);
    // In a real implementation, this would use chokidar or similar to watch files
    // and re-index when changes occur
  }

  stopWatching(): void {
    if (this.watcher) {
      // Stop the actual watcher
      this.watcher = null;
    }
  }
}

// Scheduler for periodic indexing
class IndexScheduler {
  private indexer: DocumentationIndexer;
  private scheduledTask: any | null = null; // Using any since we're not implementing actual scheduler

  constructor(indexer: DocumentationIndexer) {
    this.indexer = indexer;
  }

  scheduleIndexing(cronExpression: string = '0 2 * * *'): void { // Daily at 2 AM
    console.log(`Scheduling indexing to run: ${cronExpression}`);
    // In a real implementation, this would use node-cron or similar to schedule indexing
  }

  stopScheduling(): void {
    if (this.scheduledTask) {
      // Stop the actual scheduled task
      this.scheduledTask = null;
    }
  }
}

export { DocumentationIndexer, ContentWatcher, IndexScheduler };
export type { DocumentChunk };