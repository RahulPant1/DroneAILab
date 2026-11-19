// Assistant state shared across tabs: which runs are attached, the chat, and a pending "apply".

import { useSyncExternalStore } from 'react';
import type { Params } from './lab';

export type RunRef =
  | { saved_id: string; label: string }
  | { experiment_id: string; params: Params; seed: number; label: string };

export interface ChatMessage {
  role: 'user' | 'assistant' | 'error';
  content: string;
  runs?: string[];
  unverified?: string[];
  seconds?: number;
}

export interface SuggestionDiff {
  param: string;
  label: string;
  unit: string;
  from: number | string;
  to: number | string;
}

export interface Suggestion {
  experiment_id: string;
  experiment_title: string;
  params: Params;
  diff: SuggestionDiff[];
  reason: string;
  expected_effect: string;
  unverified_numbers: string[];
}

interface State {
  context: { source: string; refs: RunRef[] } | null; // what the assistant can "see"
  messages: ChatMessage[];
  suggestion: Suggestion | null;
  pendingApply: { experiment_id: string; params: Params } | null; // set by Apply, consumed by the Lab
  open: boolean;
}

let state: State = { context: null, messages: [], suggestion: null, pendingApply: null, open: false };
const listeners = new Set<() => void>();

function set(patch: Partial<State>): void {
  state = { ...state, ...patch };
  listeners.forEach((fn) => fn());
}

export const assistantStore = {
  get: () => state,
  subscribe: (fn: () => void) => {
    listeners.add(fn);
    return () => listeners.delete(fn);
  },
  setContext: (source: string, refs: RunRef[]) => set({ context: refs.length ? { source, refs } : null }),
  clearContext: () => set({ context: null }),
  addMessage: (m: ChatMessage) => set({ messages: [...state.messages.slice(-29), m] }),
  clearChat: () => set({ messages: [], suggestion: null }),
  setSuggestion: (s: Suggestion | null) => set({ suggestion: s }),
  apply: (s: Suggestion) => set({ pendingApply: { experiment_id: s.experiment_id, params: s.params }, suggestion: null }),
  takePending: () => {
    const p = state.pendingApply;
    if (p) set({ pendingApply: null });
    return p;
  },
  setOpen: (open: boolean) => set({ open }),
};

export function useAssistant(): State {
  return useSyncExternalStore(assistantStore.subscribe, assistantStore.get);
}

// ---- API ----

async function post<T>(url: string, body: unknown): Promise<T> {
  const resp = await fetch(url, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) });
  if (!resp.ok) {
    let detail = `HTTP ${resp.status}`;
    try {
      detail = (await resp.json()).detail ?? detail;
    } catch {
      /* not JSON */
    }
    throw new Error(detail);
  }
  return resp.json() as Promise<T>;
}

const wireRefs = (refs: RunRef[]) =>
  refs.map((r) => ('saved_id' in r ? { saved_id: r.saved_id, label: r.label } : { experiment_id: r.experiment_id, params: r.params, seed: r.seed, label: r.label }));

export const assistantApi = {
  ask: (question: string, refs: RunRef[], history: { role: string; content: string }[]) =>
    post<{ answer: string; runs: string[]; unverified_numbers: string[]; duration_s: number }>('/api/assistant/ask', {
      question,
      context: wireRefs(refs),
      history,
    }),
  suggest: (refs: RunRef[], goal: string) =>
    post<{ ok: boolean; suggestion?: Suggestion; error?: string; raw?: string; duration_s: number }>('/api/assistant/suggest', {
      context: wireRefs(refs),
      goal,
    }),
};
