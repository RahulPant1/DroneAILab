// 🎙 Voice commands: press the mic, talk, press again (or it stops by itself after 8 s). The recording goes to the
// DroneLab server only (offline Whisper on this laptop), becomes text, and the server turns it into actions.
// You can also type a command. Replies are spoken with the browser's own voices (offline ones when it has them).

import { useEffect, useRef, useState } from 'react';
import { api, sim, useSimUi } from './sim';

const EXAMPLES = ['take off', 'go left 3 metres', 'turn right', 'come home', 'race the figure eight with the autopilot',
  'bring two wingmen in a V', 'explore the building without GPS', 'stop', 'land'];

function speak(text: string) {
  if (!('speechSynthesis' in window)) return;
  window.speechSynthesis.cancel();
  const u = new SpeechSynthesisUtterance(text.replace(/[“”]/g, ''));
  const voices = window.speechSynthesis.getVoices().filter((v) => v.lang.startsWith('en'));
  u.voice = voices.find((v) => v.localService && /India/i.test(v.name)) ?? voices.find((v) => v.localService) ?? voices[0] ?? null;
  u.rate = 1.05;
  window.speechSynthesis.speak(u);
}

// ---- listening: voice-activity detection in the browser ----
// The mic level is checked 20 times a second. Speech starts a recording (with the last 0.3 s kept, so the first
// word isn't clipped); 0.6 s of quiet ends it and sends it. While the drone is talking, it doesn't listen.
const PREROLL_S = 0.3;
const END_SILENCE_MS = 600;
const MIN_SPEECH_MS = 250;
const MAX_UTTERANCE_MS = 5000;

function encodeWav(chunks: Float32Array[], rate: number, outRate = 16000): Blob {
  const total = chunks.reduce((n, c) => n + c.length, 0);
  const all = new Float32Array(total);
  let o = 0;
  for (const c of chunks) { all.set(c, o); o += c.length; }
  const step = rate / outRate;
  const n = Math.floor(total / step);
  const buf = new ArrayBuffer(44 + n * 2);
  const v = new DataView(buf);
  const w = (off: number, str: string) => [...str].forEach((ch, k) => v.setUint8(off + k, ch.charCodeAt(0)));
  w(0, 'RIFF'); v.setUint32(4, 36 + n * 2, true); w(8, 'WAVE'); w(12, 'fmt ');
  v.setUint32(16, 16, true); v.setUint16(20, 1, true); v.setUint16(22, 1, true); v.setUint32(24, outRate, true);
  v.setUint32(28, outRate * 2, true); v.setUint16(32, 2, true); v.setUint16(34, 16, true); w(36, 'data'); v.setUint32(40, n * 2, true);
  for (let k = 0; k < n; k++) {
    const x = Math.max(-1, Math.min(1, all[Math.floor(k * step)]));
    v.setInt16(44 + k * 2, x < 0 ? x * 0x8000 : x * 0x7fff, true);
  }
  return new Blob([buf], { type: 'audio/wav' });
}

interface Listener {
  stop: () => void;
}

/** Start listening. `onUtterance` gets each spoken command as a WAV; `once` = stop after the first one. */
async function listen(onUtterance: (wav: Blob) => void, onLevel: (talking: boolean) => void, once: boolean,
                      onEnd: () => void): Promise<Listener> {
  const stream = await navigator.mediaDevices.getUserMedia({ audio: { echoCancellation: true, noiseSuppression: true, autoGainControl: false } });
  const ctx = new AudioContext();
  const src = ctx.createMediaStreamSource(stream);
  const proc = ctx.createScriptProcessor(2048, 1, 1);
  const rate = ctx.sampleRate;
  const preroll: Float32Array[] = [];
  let preLen = 0;
  let speech: Float32Array[] | null = null;
  let started = 0, lastLoud = 0, floor = 0.01, peak = 0, stopped = false;
  const finish = () => {
    if (stopped) return;
    stopped = true;
    proc.disconnect(); src.disconnect();
    stream.getTracks().forEach((t) => t.stop());
    ctx.close().catch(() => undefined);
    onEnd();
  };
  proc.onaudioprocess = (e) => {
    const frame = new Float32Array(e.inputBuffer.getChannelData(0));
    const now = performance.now();
    if (window.speechSynthesis?.speaking) { speech = null; onLevel(false); return; } // don't hear itself
    let sum = 0;
    for (let k = 0; k < frame.length; k++) sum += frame[k] * frame[k];
    const rms = Math.sqrt(sum / frame.length);
    // The room's background level: drops quickly to anything quieter, creeps up slowly (even while talking), so a
    // steady fan or hum can't hold the recording open for ever.
    floor = rms < floor ? 0.7 * floor + 0.3 * rms : floor + 0.002 * (rms - floor);
    const threshold = Math.max(0.012, floor * 3);
    // While talking, "quiet again" = well below how loud your voice was, not just below the start threshold.
    const loud = speech ? rms > Math.max(threshold, 0.2 * peak) : rms > threshold;
    if (!speech) {
      preroll.push(frame); preLen += frame.length;
      while (preLen - preroll[0].length > PREROLL_S * rate) preLen -= preroll.shift()!.length;
      if (loud) { speech = [...preroll]; started = now; lastLoud = now; peak = rms; onLevel(true); }
      return;
    }
    speech.push(frame);
    peak = Math.max(peak, rms);
    if (loud) lastLoud = now;
    if (now - lastLoud > END_SILENCE_MS || now - started > MAX_UTTERANCE_MS) {
      const got = speech;
      speech = null; preroll.length = 0; preLen = 0;
      onLevel(false);
      if (lastLoud - started >= MIN_SPEECH_MS) {
        onUtterance(encodeWav(got, rate));
        if (once) finish();
      }
    }
  };
  src.connect(proc);
  proc.connect(ctx.destination);
  return { stop: finish };
}

/** The microphone + command logic, shared by the engineer's voice bar and the 🎮 Play screen.
 *  Press once: it listens, sends what you said when you stop talking, and stops. Hands-free: it keeps listening. */
export function useVoice() {
  const { status } = useSimUi();
  const [mic, setMic] = useState<{ ok: boolean; detail: string } | null>(null);
  const [listening, setListening] = useState(false);
  const [talking, setTalking] = useState(false);
  const [busy, setBusy] = useState('');
  const [heardText, setHeardText] = useState('');
  const [talk, setTalk] = useState(() => {
    try { return localStorage.getItem('dronelab.voice.talk') !== '0'; } catch { return true; }
  });
  const [handsFree, setHandsFree] = useState(false);
  const listener = useRef<Listener | null>(null);
  const seen = useRef(sim.voiceSeq);
  const offline = status !== 'open';

  useEffect(() => {
    api.voiceStatus().then((r) => setMic({ ok: r.available, detail: r.detail })).catch(() => setMic({ ok: false, detail: 'Server not reachable' }));
    return () => listener.current?.stop();
  }, []);
  useEffect(() => {
    if (sim.voiceSeq !== seen.current && sim.voice) {
      seen.current = sim.voiceSeq;
      setBusy('');
      if (talk) speak(sim.voice.short || sim.voice.say);
    }
  });

  const send = (text: string) => {
    if (!text.trim()) return;
    setBusy('thinking…');
    sim.send('voice', { text });
  };
  const onUtterance = async (wav: Blob) => {
    setBusy('listening…');
    try {
      const res = await api.voiceTranscribe(wav);
      if (!res.text) { setBusy(''); return; }
      setHeardText(res.text);
      send(res.text);
    } catch (err) {
      setBusy(`🎙 ${(err as Error).message}`);
    }
  };
  const start = async (continuous: boolean) => {
    api.voiceWarmup(); // load Whisper + the AI now, so the answer comes quickly
    try {
      listener.current = await listen(onUtterance, setTalking, !continuous, () => { setListening(false); setTalking(false); listener.current = null; });
      setListening(true);
    } catch (err) {
      setBusy(`🎙 Microphone not available: ${(err as Error).message}`);
    }
  };
  const stop = () => listener.current?.stop();
  const toggle = () => (listening ? stop() : start(handsFree));
  const setHandsFreeOn = (on: boolean) => {
    setHandsFree(on);
    if (listening) stop();
    if (on) start(true);
  };
  const setTalkSaved = (on: boolean) => {
    setTalk(on);
    try { localStorage.setItem('dronelab.voice.talk', on ? '1' : '0'); } catch { /* private window */ }
  };
  const canTalk = !offline && !!mic?.ok;
  return { mic, recording: listening, talking, busy, heardText, talk, setTalk: setTalkSaved, handsFree,
           setHandsFree: setHandsFreeOn, send, toggle, canTalk, offline, reply: sim.voice };
}

export default function VoiceBar({ onLearn }: { onLearn: (slug: string) => void }) {
  const { mic, recording, talking, busy, heardText, talk, setTalk, handsFree, setHandsFree, send, toggle, canTalk, offline } = useVoice();
  const [typed, setTyped] = useState('');
  useEffect(() => { if (heardText) setTyped(heardText); }, [heardText]);
  const v = sim.voice;
  return (
    <div className="voice-bar">
      <div className="row">
        <button className={`primary big voice-mic ${recording ? 'recording' : ''}`} disabled={!canTalk && !recording}
          onClick={toggle}
          title={mic?.ok ? 'Press and say a command: it is sent when you stop talking. Heard on this laptop only.' : mic?.detail}>
          {recording ? (talking ? '🔴 hearing…' : '👂 listening') : '🎙 Talk'}
        </button>
        <label className="inline" title="Keep listening: every time you stop talking, the command is sent. No buttons.">
          <input type="checkbox" checked={handsFree} disabled={!canTalk} onChange={(e) => setHandsFree(e.target.checked)} /> hands-free
        </label>
        <form className="voice-type" onSubmit={(e) => { e.preventDefault(); send(typed); }}>
          <input value={typed} placeholder="…or type: race the figure eight with the autopilot" disabled={offline}
            onChange={(e) => setTyped(e.target.value)} list="voice-examples" />
          <datalist id="voice-examples">{EXAMPLES.map((x) => <option key={x} value={x} />)}</datalist>
          <button type="submit" disabled={offline || !typed.trim()}>Send</button>
        </form>
        {v?.pending && (
          <>
            <button className="primary" disabled={offline} onClick={() => send('go')}>▶ Go</button>
            <button disabled={offline} onClick={() => send('cancel')}>Cancel</button>
          </>
        )}
        <button className="danger" disabled={offline} onClick={() => send('stop')} title="Same as saying stop: hover now, cancel what's waiting">✋ Stop</button>
        <label className="inline" title="Say the answers out loud">
          <input type="checkbox" checked={talk} onChange={(e) => setTalk(e.target.checked)} /> 🔊
        </label>
        <button className="link" onClick={() => onLearn('27-voice-commands')}>📖 Learn: voice</button>
      </div>
      <p className="small voice-reply">
        {busy ? <span className="muted">{busy}</span> : v ? (
          <>
            <span className="muted">heard “{v.heard}”{v.source === 'ai' ? ' (AI)' : ''} → </span>
            <b>{v.say}</b>
          </>
        ) : (
          <span className="muted">{mic && !mic.ok ? mic.detail : 'Try: “take off”, “go left 3 metres”, “race the figure eight with the autopilot”, “stop”.'}</span>
        )}
      </p>
    </div>
  );
}
