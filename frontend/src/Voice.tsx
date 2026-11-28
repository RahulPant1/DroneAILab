// 🎙 Voice commands: press the mic, talk, press again (or it stops by itself after 8 s). The recording goes to the
// DroneLab server only (offline Whisper on this laptop), becomes text, and the server turns it into actions.
// You can also type a command. Replies are spoken with the browser's own voices (offline ones when it has them).

import { useEffect, useRef, useState } from 'react';
import { api, sim, useSimUi } from './sim';

const MAX_RECORD_MS = 8000;
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

export default function VoiceBar({ onLearn }: { onLearn: (slug: string) => void }) {
  const { status } = useSimUi();
  const [mic, setMic] = useState<{ ok: boolean; detail: string } | null>(null);
  const [recording, setRecording] = useState(false);
  const [busy, setBusy] = useState('');
  const [typed, setTyped] = useState('');
  const [talk, setTalk] = useState(() => {
    try { return localStorage.getItem('dronelab.voice.talk') !== '0'; } catch { return true; }
  });
  const rec = useRef<MediaRecorder | null>(null);
  const seen = useRef(sim.voiceSeq);
  const offline = status !== 'open';

  useEffect(() => {
    api.voiceStatus().then((r) => setMic({ ok: r.available, detail: r.detail })).catch(() => setMic({ ok: false, detail: 'Server not reachable' }));
  }, []);
  // speak each new answer once
  useEffect(() => {
    if (sim.voiceSeq !== seen.current && sim.voice) {
      seen.current = sim.voiceSeq;
      setBusy('');
      if (talk) speak(sim.voice.say);
    }
  });

  const send = (text: string) => {
    if (!text.trim()) return;
    setBusy('thinking…');
    sim.send('voice', { text });
  };

  const start = async () => {
    try {
      const stream = await navigator.mediaDevices.getUserMedia({ audio: { echoCancellation: true, noiseSuppression: true } });
      const type = MediaRecorder.isTypeSupported('audio/webm;codecs=opus') ? 'audio/webm;codecs=opus' : '';
      const r = new MediaRecorder(stream, type ? { mimeType: type } : undefined);
      const chunks: Blob[] = [];
      r.ondataavailable = (e) => e.data.size && chunks.push(e.data);
      r.onstop = async () => {
        stream.getTracks().forEach((t) => t.stop());
        setRecording(false);
        setBusy('listening…');
        try {
          const res = await api.voiceTranscribe(new Blob(chunks, { type: r.mimeType || 'audio/webm' }));
          if (!res.text) {
            setBusy('');
            if (talk) speak("I didn't hear anything.");
            return;
          }
          setTyped(res.text);
          send(res.text);
        } catch (err) {
          setBusy(`🎙 ${(err as Error).message}`);
        }
      };
      rec.current = r;
      r.start();
      setRecording(true);
      window.setTimeout(() => r.state === 'recording' && r.stop(), MAX_RECORD_MS);
    } catch (err) {
      setBusy(`🎙 Microphone not available: ${(err as Error).message}`);
    }
  };
  const stop = () => rec.current?.state === 'recording' && rec.current.stop();

  const v = sim.voice;
  return (
    <div className="voice-bar">
      <div className="row">
        <button className={`primary big voice-mic ${recording ? 'recording' : ''}`} disabled={offline || !mic?.ok || busy === 'listening…' || busy === 'thinking…'}
          onClick={() => (recording ? stop() : start())}
          title={mic?.ok ? 'Press, say a command, press again (stops by itself after 8 s). Heard on this laptop only.' : mic?.detail}>
          {recording ? '⏹ Done' : '🎙 Talk'}
        </button>
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
          <input type="checkbox" checked={talk} onChange={(e) => {
            setTalk(e.target.checked);
            try { localStorage.setItem('dronelab.voice.talk', e.target.checked ? '1' : '0'); } catch { /* private window */ }
          }} /> 🔊
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
