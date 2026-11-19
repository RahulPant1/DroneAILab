// The AI side panel. Works on every tab; shows exactly which runs the AI can see.

import { useEffect, useRef, useState } from 'react';
import { assistantApi, assistantStore, useAssistant, type Suggestion } from './assistant';
import { formatValue } from './lab';
import { renderMarkdown } from './Learn';
import type { Health } from './sim';

const QUICK_QUESTIONS = ['Explain what happened in this run.', 'Why did it behave like this?', 'What should I look at in the plots?'];

function SuggestionCard({ s, onApply }: { s: Suggestion; onApply: () => void }) {
  return (
    <div className="suggestion">
      <div className="suggestion-head">💡 Suggested: <b>{s.experiment_title}</b></div>
      <table className="kv diff">
        <thead><tr><th>Setting</th><th>now</th><th></th><th>suggested</th></tr></thead>
        <tbody>
          {s.diff.map((d) => (
            <tr key={d.param}><th>{d.label}</th><td>{formatValue(d.from)}</td><td>→</td><td><b>{formatValue(d.to)}</b> <span className="unit">{d.unit}</span></td></tr>
          ))}
        </tbody>
      </table>
      <p><b>Why:</b> {s.reason}</p>
      <p><b>Prediction</b> <span className="tag tag-hypothesis">Hypothesis</span> {s.expected_effect}</p>
      {s.unverified_numbers.length > 0 && <p className="warn small">⚠ Numbers not found in the run data: {s.unverified_numbers.join(', ')}</p>}
      <p className="muted small">Checked: valid experiment, real settings, all values inside the allowed limits. Nothing runs until you click Apply.</p>
      <div className="row">
        <button className="primary" onClick={onApply}>Apply &amp; run in Lab</button>
        <button onClick={() => assistantStore.setSuggestion(null)}>Dismiss</button>
      </div>
    </div>
  );
}

export default function Assistant({ health, onApplied }: { health: Health | null; onApplied: () => void }) {
  const { context, messages, suggestion } = useAssistant();
  const [question, setQuestion] = useState('');
  const [goal, setGoal] = useState('');
  const [busy, setBusy] = useState<'' | 'ask' | 'suggest'>('');
  const endRef = useRef<HTMLDivElement>(null);
  const online = health?.llm === 'ok';
  const refs = context?.refs ?? [];

  useEffect(() => endRef.current?.scrollIntoView({ behavior: 'smooth' }), [messages, suggestion, busy]);

  const ask = async (q: string) => {
    q = q.trim();
    if (!q || busy) return;
    const history = messages.filter((m) => m.role !== 'error').slice(-4).map((m) => ({ role: m.role, content: m.content }));
    assistantStore.addMessage({ role: 'user', content: q, runs: refs.map((r) => r.label) });
    setQuestion('');
    setBusy('ask');
    try {
      const res = await assistantApi.ask(q, refs, history);
      assistantStore.addMessage({ role: 'assistant', content: res.answer, runs: res.runs, unverified: res.unverified_numbers, seconds: res.duration_s });
    } catch (e) {
      assistantStore.addMessage({ role: 'error', content: (e as Error).message });
    } finally {
      setBusy('');
    }
  };

  const suggest = async () => {
    if (busy) return;
    setBusy('suggest');
    assistantStore.setSuggestion(null);
    try {
      const res = await assistantApi.suggest(refs, goal);
      if (res.ok && res.suggestion) assistantStore.setSuggestion(res.suggestion);
      else assistantStore.addMessage({ role: 'error', content: `Suggestion rejected: ${res.error}` });
    } catch (e) {
      assistantStore.addMessage({ role: 'error', content: (e as Error).message });
    } finally {
      setBusy('');
    }
  };

  return (
    <aside className="assistant">
      <header className="assistant-head">
        <h2>💬 Assistant</h2>
        <span className={`pill ${online ? 'ok' : 'off'}`}>{online ? health?.model : 'AI offline'}</span>
        <span className="spacer" />
        <button className="small-btn" onClick={() => assistantStore.clearChat()} title="Clear the conversation">Clear</button>
        <button className="small-btn" onClick={() => assistantStore.setOpen(false)} aria-label="Close">✕</button>
      </header>

      <div className="assistant-context">
        <span className="muted small">The AI can see:</span>{' '}
        {refs.length === 0 ? <span className="small">no run (general questions only)</span> : refs.map((r) => <span key={r.label} className="chip">{r.label}</span>)}
        {refs.length > 0 && <button className="link small" onClick={() => assistantStore.clearContext()}>detach</button>}
        <div className="muted tiny">{context ? `from ${context.source}` : 'Run an experiment in the Lab, or open/compare runs in History, to attach them.'} Only a short summary is sent (settings, metrics, events), never raw data.</div>
      </div>

      {!online && (
        <div className="offline-note">
          AI is offline (Ollama isn't reachable, or the model isn't installed). Everything else in DroneLab still works.
        </div>
      )}

      <div className="assistant-messages">
        {messages.length === 0 && (
          <div className="muted small">
            Ask about a run. Answers label each claim: <span className="tag tag-measured">Measured</span> means from your run data, and{' '}
            <span className="tag tag-hypothesis">Hypothesis</span> means the AI's reasoning. The first answer can take up to a minute while the model loads; after that it's a few seconds.
          </div>
        )}
        {messages.map((m, i) => (
          <div key={i} className={`msg msg-${m.role}`}>
            {m.role === 'assistant' ? renderMarkdown(m.content) : <p>{m.content}</p>}
            {m.role === 'assistant' && m.unverified && m.unverified.length > 0 && (
              <p className="warn small">⚠ These numbers aren't in the run data, so treat them with care: {m.unverified.join(', ')}</p>
            )}
            {m.role === 'assistant' && <p className="muted tiny">{m.runs?.length ? `Based on: ${m.runs.join(', ')}` : 'No run attached'} · {m.seconds} s</p>}
          </div>
        ))}
        {busy && <div className="msg msg-assistant muted">{busy === 'ask' ? 'Thinking…' : 'Designing an experiment…'}</div>}
        {suggestion && <SuggestionCard s={suggestion} onApply={() => { assistantStore.apply(suggestion); onApplied(); }} />}
        <div ref={endRef} />
      </div>

      <div className="assistant-input">
        {refs.length > 0 && (
          <div className="row quick">
            {QUICK_QUESTIONS.map((q) => <button key={q} className="small-btn" disabled={!online || !!busy} onClick={() => ask(q)}>{q}</button>)}
          </div>
        )}
        <form className="row" onSubmit={(e) => { e.preventDefault(); ask(question); }}>
          <input className="grow" placeholder={online ? 'Ask about this run…' : 'AI offline'} value={question} maxLength={1000}
            disabled={!online || !!busy} onChange={(e) => setQuestion(e.target.value)} />
          <button className="primary" disabled={!online || !!busy || !question.trim()}>Ask</button>
        </form>
        <div className="row">
          <input className="grow" placeholder="Goal for a suggestion (optional), e.g. 'less overshoot'" value={goal} maxLength={300}
            disabled={!online || !!busy} onChange={(e) => setGoal(e.target.value)} />
          <button disabled={!online || !!busy} onClick={suggest}>💡 Suggest an experiment</button>
        </div>
      </div>
    </aside>
  );
}
