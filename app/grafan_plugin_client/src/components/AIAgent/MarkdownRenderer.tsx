import React from 'react';
import { css } from '@emotion/css';
import { useTheme2 } from '@grafana/ui';
import { GrafanaTheme2 } from '@grafana/data';

interface MarkdownRendererProps {
  content: string;
}

const getStyles = (theme: GrafanaTheme2) => ({
  container: css`
    & {
      color: ${theme.colors.text.primary};
    }
    
    p {
      margin: ${theme.spacing(1)} 0;
      
      &:first-child {
        margin-top: 0;
      }
      
      &:last-child {
        margin-bottom: 0;
      }
    }
    
    ul, ol {
      margin: ${theme.spacing(1)} 0;
      padding-left: ${theme.spacing(3)};
      
      li {
        margin-bottom: ${theme.spacing(0.5)};
      }
    }
    
    code {
      background: ${theme.colors.background.secondary};
      padding: 2px 6px;
      border-radius: 3px;
      font-family: ${theme.typography.fontFamilyMonospace};
      font-size: 0.9em;
      color: ${theme.colors.text.primary};
    }
    
    pre {
      background: ${theme.colors.background.secondary};
      padding: ${theme.spacing(1)};
      border-radius: ${theme.shape.radius.default};
      overflow-x: auto;
      margin: ${theme.spacing(1)} 0;
      border: 1px solid ${theme.colors.border.weak};
      
      code {
        background: none;
        padding: 0;
        color: ${theme.colors.text.primary};
      }
    }
    
    blockquote {
      border-left: 4px solid ${theme.colors.border.medium};
      margin: ${theme.spacing(1)} 0;
      padding: ${theme.spacing(1)} ${theme.spacing(2)};
      color: ${theme.colors.text.secondary};
      background: ${theme.colors.background.secondary};
      border-radius: ${theme.shape.radius.default};
    }
    
    h1, h2, h3, h4, h5, h6 {
      margin: ${theme.spacing(1.5)} 0 ${theme.spacing(0.5)} 0;
      font-weight: ${theme.typography.fontWeightBold};
      color: ${theme.colors.text.primary};
    }
    
    h1 {
      font-size: 1.5em;
      border-bottom: 1px solid ${theme.colors.border.weak};
      padding-bottom: ${theme.spacing(0.5)};
    }
    
    h2 {
      font-size: 1.3em;
      border-bottom: 1px solid ${theme.colors.border.weak};
      padding-bottom: ${theme.spacing(0.25)};
    }
    
    h3 {
      font-size: 1.1em;
    }
    
    strong, b {
      font-weight: ${theme.typography.fontWeightBold};
      color: ${theme.colors.text.primary};
    }
    
    em, i {
      font-style: italic;
    }
    
    a {
      color: ${theme.colors.primary.main};
      text-decoration: none;
      
      &:hover {
        text-decoration: underline;
      }
    }
    
    hr {
      border: none;
      border-top: 1px solid ${theme.colors.border.weak};
      margin: ${theme.spacing(2)} 0;
    }
    
    table {
      border-collapse: collapse;
      width: 100%;
      margin: ${theme.spacing(1)} 0;
      
      th, td {
        border: 1px solid ${theme.colors.border.weak};
        padding: ${theme.spacing(0.75)} ${theme.spacing(1)};
        text-align: left;
      }
      
      th {
        background: ${theme.colors.background.secondary};
        font-weight: ${theme.typography.fontWeightBold};
      }
      
      tr:nth-child(even) {
        background: ${theme.colors.background.secondary};
      }
    }
  `,
});

/**
 * Simple markdown-to-HTML renderer
 * Converts markdown text to React elements with proper styling
 */
export const MarkdownRenderer: React.FC<MarkdownRendererProps> = ({ content }) => {
  const theme = useTheme2();
  const styles = getStyles(theme);

  // Parse markdown and convert to React elements
  const parseMarkdown = (text: string): React.ReactNode[] => {
    const lines = text.split('\n');
    const elements: React.ReactNode[] = [];
    let currentList: string[] = [];
    let inCodeBlock = false;
    let codeContent = '';

    const flushList = (type: 'ul' | 'ol') => {
      if (currentList.length > 0) {
        const ListComponent = type === 'ul' ? 'ul' : 'ol';
        elements.push(
          React.createElement(
            ListComponent,
            { key: `list-${elements.length}` },
            currentList.map((item, idx) =>
              React.createElement('li', { key: `item-${idx}` }, parseInlineMarkdown(item))
            )
          )
        );
        currentList = [];
      }
    };

    for (let i = 0; i < lines.length; i++) {
      const line = lines[i];

      // Handle code blocks
      if (line.trim().startsWith('```')) {
        if (!inCodeBlock) {
          flushList(line.includes('-') || line.includes('*') ? 'ul' : 'ol');
          inCodeBlock = true;
        } else {
          elements.push(
            React.createElement('pre', { key: `code-${elements.length}` }, codeContent.trimEnd())
          );
          inCodeBlock = false;
          codeContent = '';
        }
        continue;
      }

      if (inCodeBlock) {
        codeContent += line + '\n';
        continue;
      }

      // Headers
      if (line.match(/^#+\s/)) {
        flushList(line.includes('-') || line.includes('*') ? 'ul' : 'ol');
        const level = line.match(/^#+/)?.[0].length || 1;
        const text = line.replace(/^#+\s/, '');
        const HeaderTag = `h${Math.min(level, 6)}` as keyof JSX.IntrinsicElements;
        elements.push(
          React.createElement(HeaderTag, { key: `header-${elements.length}` }, parseInlineMarkdown(text))
        );
        continue;
      }

      // Horizontal rule
      if (line.trim().match(/^(---|===|\*\*\*|___)$/)) {
        flushList(line.includes('-') || line.includes('*') ? 'ul' : 'ol');
        elements.push(React.createElement('hr', { key: `hr-${elements.length}` }));
        continue;
      }

      // Blockquote
      if (line.trim().startsWith('>')) {
        flushList(line.includes('-') || line.includes('*') ? 'ul' : 'ol');
        const quoteContent = line.replace(/^>\s?/, '');
        elements.push(
          React.createElement('blockquote', { key: `quote-${elements.length}` }, parseInlineMarkdown(quoteContent))
        );
        continue;
      }

      // Unordered list
      if (line.match(/^[\s]*[-*+]\s/)) {
        const trimmed = line.trim();
        const content = trimmed.replace(/^[-*+]\s/, '');
        if (currentList.length === 0) {
          // Starting a new list
          if (elements.length > 0 && typeof elements[elements.length - 1] === 'object' && 
              (elements[elements.length - 1] as any).type === 'ol') {
            // Was an ordered list, flush it
            flushList('ol');
          }
        }
        currentList.push(content);
        continue;
      }

      // Ordered list
      if (line.match(/^[\s]*\d+\.\s/)) {
        const match = line.match(/^[\s]*(\d+\.\s)/);
        const content = line.replace(match?.[1] || '', '');
        if (currentList.length === 0) {
          // Starting a new list
          if (elements.length > 0 && typeof elements[elements.length - 1] === 'object' && 
              (elements[elements.length - 1] as any).type === 'ul') {
            // Was an unordered list, flush it
            flushList('ul');
          }
        }
        currentList.push(content);
        continue;
      }

      // Empty line - flush current list if any
      if (line.trim() === '') {
        if (currentList.length > 0) {
          const isOrdered = elements.length > 0 && typeof elements[elements.length - 1] === 'object' &&
                           (elements[elements.length - 1] as any).type === 'ol';
          flushList(isOrdered ? 'ol' : 'ul');
        }
        // Add paragraph break
        if (elements.length > 0 && elements[elements.length - 1] !== '') {
          // Skip extra breaks
        }
        continue;
      }

      // Regular paragraph
      flushList(line.includes('-') || line.includes('*') ? 'ul' : 'ol');
      elements.push(
        React.createElement('p', { key: `para-${elements.length}` }, parseInlineMarkdown(line))
      );
    }

    flushList(currentList.length > 0 ? 'ul' : 'ol');

    return elements;
  };

  /**
   * Parse inline markdown (bold, italic, code, links)
   */
  const parseInlineMarkdown = (text: string): React.ReactNode[] => {
    const elements: React.ReactNode[] = [];
    const patterns = [
      { pattern: /\*\*\*(.+?)\*\*\*/g, element: 'strong' }, // Bold italic
      { pattern: /\*\*(.+?)\*\*/g, element: 'strong' }, // Bold
      { pattern: /__(.+?)__/g, element: 'strong' }, // Bold alt
      { pattern: /\*(.+?)\*/g, element: 'em' }, // Italic
      { pattern: /_(.+?)_/g, element: 'em' }, // Italic alt
      { pattern: /`([^`]+)`/g, element: 'code' }, // Inline code
      { pattern: /\[(.+?)\]\((.+?)\)/g, element: 'link' }, // Links
    ];

    // Simple regex-based parsing
    let lastIndex = 0;

    // Find all matches
    const allMatches: Array<{ index: number; end: number; type: string; content: string; url?: string }> = [];

    for (const { pattern, element } of patterns) {
      let match;
      const regex = new RegExp(pattern);
      while ((match = regex.exec(text)) !== null) {
        if (element === 'link') {
          allMatches.push({
            index: match.index,
            end: match.index + match[0].length,
            type: element,
            content: match[1],
            url: match[2],
          });
        } else {
          allMatches.push({
            index: match.index,
            end: match.index + match[0].length,
            type: element,
            content: match[1],
          });
        }
      }
    }

    // Sort by index
    allMatches.sort((a, b) => a.index - b.index);

    // Build elements, avoiding overlaps
    lastIndex = 0;
    for (const match of allMatches) {
      if (match.index < lastIndex) continue; // Skip overlapping

      if (match.index > lastIndex) {
        elements.push(text.substring(lastIndex, match.index));
      }

      switch (match.type) {
        case 'strong':
          elements.push(React.createElement('strong', { key: `strong-${elements.length}` }, match.content));
          break;
        case 'em':
          elements.push(React.createElement('em', { key: `em-${elements.length}` }, match.content));
          break;
        case 'code':
          elements.push(React.createElement('code', { key: `code-${elements.length}` }, match.content));
          break;
        case 'link':
          elements.push(
            React.createElement(
              'a',
              { key: `link-${elements.length}`, href: match.url, target: '_blank', rel: 'noopener noreferrer' },
              match.content
            )
          );
          break;
      }

      lastIndex = match.end;
    }

    if (lastIndex < text.length) {
      elements.push(text.substring(lastIndex));
    }

    return elements.length > 0 ? elements : [text];
  };

  const parsed = parseMarkdown(content);

  return <div className={styles.container}>{parsed}</div>;
};
