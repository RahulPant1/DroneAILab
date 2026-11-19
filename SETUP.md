# DroneLab: how to start (5 minutes)

## 1. WSL memory (decided: leave as is)
`.wslconfig` stays at `memory=9GB`, `swap=0`. DroneLab is built to fit in that space.
Rule of thumb: don't run ClipForge (or other GPU/RAM-heavy apps) while using the AI assistant.

## 2. Project folder (done)
```bash
cd ~/DroneAILab
ls -a        # CLAUDE.md  PROGRESS.md  docs/  backend/  .claude/  .gitignore  .env.example  SETUP.md
```

## 3. Snapshot your current setup (so you can verify nothing changed)
```bash
mkdir -p ~/pre_dronelab
ollama list   > ~/pre_dronelab/ollama.txt
conda env list > ~/pre_dronelab/conda.txt 2>/dev/null
pip list      > ~/pre_dronelab/pip.txt 2>/dev/null
npm ls -g --depth=0 > ~/pre_dronelab/npm_global.txt 2>/dev/null
```

## 4. Start Claude Code in the project
```bash
cd ~/DroneAILab
claude
```
- Type `/permissions` once to confirm the deny rules from `.claude/settings.json` are loaded.
- Do **not** use auto-accept or `--dangerously-skip-permissions`. Read every shell command before you approve it.

## 5. First prompt (copy-paste)
> Read docs/PRD.md and CLAUDE.md fully. Do **Phase 0 only**: run the environment checks, then report findings, the proposed env choice (venv vs conda), the exact project-local install commands, and the detected Gemma model tag. Do not install or create anything yet. Wait for my approval.

## 6. After each phase
- Follow the demo script Claude Code gives you.
- Read the files in the code walkthrough (this is where the learning happens).
- Then reply **"go Phase X"** to continue.

## Sanity check anytime
```bash
diff <(ollama list) ~/pre_dronelab/ollama.txt && echo "Ollama untouched"
ollama ps          # should be empty when you're not using the assistant
du -sh ~/DroneAILab
```

## Uninstall
```bash
rm -rf ~/DroneAILab      # removes everything (shared uv/npm download caches are kept on purpose)
```
