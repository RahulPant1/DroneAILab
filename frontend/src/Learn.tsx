// Shows a docs/learn/ page. A tiny Markdown renderer (headings, lists, code, bold, italic, `code`)
// keeps us free of extra dependencies. React escapes all text, so this is safe.
// A '## Answers' section is folded behind a "Show answers" button, so learners try the self-check first.

import { useEffect, useState, type ReactNode } from 'react';
import { api } from './sim';

const STAR = '\u0001'; // stands in for an escaped \* while we look for bold / italic markers
const unstar = (t: string) => t.split(STAR).join('*');

function inline(text: string): ReactNode[] {
  return text.replace(/\\\*/g, STAR).split(/(`[^`]+`|\*\*[^*]+\*\*|\*[^*\s][^*]*\*|\[Measured\]|\[Hypothesis\])/g).map((part, i) => {
    if (part === '[Measured]') return <span key={i} className="tag tag-measured" title="Read directly from the run data">Measured</span>;
    if (part === '[Hypothesis]') return <span key={i} className="tag tag-hypothesis" title="The AI's explanation or reasoning: not measured">Hypothesis</span>;
    if (part.startsWith('`') && part.endsWith('`')) return <code key={i}>{unstar(part.slice(1, -1))}</code>;
    if (part.startsWith('**') && part.endsWith('**')) return <b key={i}>{unstar(part.slice(2, -2))}</b>;
    if (part.length > 2 && part.startsWith('*') && part.endsWith('*')) return <i key={i}>{unstar(part.slice(1, -1))}</i>;
    return unstar(part);
  });
}

export function renderMarkdown(md: string): ReactNode[] {
  // Fold an "## Answers" section (up to the next ## heading) behind a button.
  const m = md.match(/^## Answers[^\n]*\n([\s\S]*?)(?=^## |(?![\s\S]))/m);
  if (m && m.index !== undefined) {
    const before = md.slice(0, m.index), after = md.slice(m.index + m[0].length);
    return [
      ...renderBlocks(before, 'a'),
      <details key="answers" className="answers"><summary>Show answers (try the questions first)</summary>{renderBlocks(m[1], 'b')}</details>,
      ...renderBlocks(after, 'c'),
    ];
  }
  return renderBlocks(md, 'a');
}

function renderBlocks(md: string, keyPrefix: string): ReactNode[] {
  const out: ReactNode[] = [];
  const lines = md.split('\n');
  let list: { ordered: boolean; items: string[] } | null = null;
  const flush = () => {
    if (!list) return;
    const items = list.items.map((t, i) => <li key={i}>{inline(t)}</li>);
    out.push(list.ordered ? <ol key={keyPrefix + out.length}>{items}</ol> : <ul key={keyPrefix + out.length}>{items}</ul>);
    list = null;
  };
  for (let i = 0; i < lines.length; i++) {
    const line = lines[i];
    if (line.startsWith('```')) {
      flush();
      const code: string[] = [];
      while (++i < lines.length && !lines[i].startsWith('```')) code.push(lines[i]);
      out.push(<pre key={keyPrefix + out.length}>{code.join('\n')}</pre>);
      continue;
    }
    const bullet = line.match(/^\s*-\s+(.*)/);
    const numbered = line.match(/^\s*\d+\.\s+(.*)/);
    if (bullet || numbered) {
      const ordered = !!numbered;
      if (!list || list.ordered !== ordered) {
        flush();
        list = { ordered, items: [] };
      }
      list.items.push((bullet ?? numbered)![1]);
      continue;
    }
    flush();
    if (line.startsWith('# ')) out.push(<h2 key={keyPrefix + out.length}>{inline(line.slice(2))}</h2>);
    else if (line.startsWith('## ')) out.push(<h3 key={keyPrefix + out.length}>{inline(line.slice(3))}</h3>);
    else if (line.startsWith('### ')) out.push(<h4 key={keyPrefix + out.length}>{inline(line.slice(4))}</h4>);
    else if (line.trim()) out.push(<p key={keyPrefix + out.length}>{inline(line)}</p>);
  }
  flush();
  return out;
}

export default function Learn({ slug, onClose }: { slug: string; onClose: () => void }) {
  const [md, setMd] = useState<string | null>(null);
  useEffect(() => {
    api.learn(slug).then((r) => setMd(r.markdown)).catch(() => setMd('# Not found\nThis page does not exist yet.'));
  }, [slug]);
  useEffect(() => {
    const esc = (e: KeyboardEvent) => e.key === 'Escape' && onClose();
    window.addEventListener('keydown', esc);
    return () => window.removeEventListener('keydown', esc);
  }, [onClose]);
  return (
    <div className="modal-backdrop" onClick={onClose}>
      <article className="modal" onClick={(e) => e.stopPropagation()}>
        <button className="close" onClick={onClose} aria-label="Close">✕</button>
        {md ? renderMarkdown(md) : <p className="muted">Loading…</p>}
      </article>
    </div>
  );
}
