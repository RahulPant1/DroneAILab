"""Talks to the already-installed Ollama over HTTP. Nothing else: no CLI, no model management.

Per-request options only (nothing global is changed):
  keep_alive "2m" -> Ollama unloads the model after 2 idle minutes, freeing the GPU;
  num_ctx 8192    -> context window;
  temperature 0.3 -> fairly factual;
  think false     -> skip Gemma's slow 'thinking' mode (it also gets in the way of JSON replies).
"""

from __future__ import annotations

import os
from dataclasses import dataclass

import httpx

TEMPERATURE = 0.3
CONNECT_TIMEOUT_S = 3.0
RESPONSE_TIMEOUT_S = 180.0  # the first request also loads the model, which can take a while


class AIOffline(RuntimeError):
    """Ollama isn't reachable, or the configured model isn't installed."""


class AIError(RuntimeError):
    """Ollama answered, but not usefully (error status, timeout, odd reply)."""


@dataclass
class OllamaClient:
    url: str
    model: str
    keep_alive: str = "2m"
    num_ctx: int = 8192
    temperature: float = TEMPERATURE
    transport: httpx.AsyncBaseTransport | None = None  # tests plug in a fake Ollama here

    @classmethod
    def from_env(cls) -> "OllamaClient":
        return cls(
            url=os.environ.get("OLLAMA_URL", "http://localhost:11434").rstrip("/"),
            model=os.environ.get("DRONELAB_LLM_MODEL", ""),
            keep_alive=os.environ.get("DRONELAB_LLM_KEEP_ALIVE", "2m"),
            num_ctx=int(os.environ.get("DRONELAB_LLM_NUM_CTX", "8192")),
        )

    def payload(self, messages: list[dict], json_schema: dict | None = None) -> dict:
        body = {
            "model": self.model,
            "messages": messages,
            "stream": False,
            "think": False,
            "keep_alive": self.keep_alive,
            "options": {"num_ctx": self.num_ctx, "temperature": self.temperature},
        }
        if json_schema is not None:
            body["format"] = json_schema
        return body

    async def chat(self, messages: list[dict], json_schema: dict | None = None) -> str:
        """Send a chat and return the reply text."""
        if not self.model:
            raise AIOffline("No model configured (set DRONELAB_LLM_MODEL in .env).")
        timeout = httpx.Timeout(RESPONSE_TIMEOUT_S, connect=CONNECT_TIMEOUT_S)
        try:
            async with httpx.AsyncClient(timeout=timeout, transport=self.transport) as client:
                resp = await client.post(f"{self.url}/api/chat", json=self.payload(messages, json_schema))
        except (httpx.ConnectError, httpx.ConnectTimeout) as exc:
            raise AIOffline(f"Can't reach Ollama at {self.url}. Is it running?") from exc
        except httpx.TimeoutException as exc:
            raise AIError("The model took too long to answer. Try again, or ask a shorter question.") from exc
        except httpx.HTTPError as exc:
            raise AIError(f"Problem talking to Ollama: {exc}") from exc

        if resp.status_code == 404:
            raise AIOffline(f"Ollama doesn't have the model '{self.model}' installed.")
        if resp.status_code != 200:
            raise AIError(f"Ollama returned an error ({resp.status_code}): {resp.text[:200]}")
        try:
            content = resp.json()["message"]["content"]
        except (ValueError, KeyError, TypeError) as exc:
            raise AIError("Ollama sent a reply in an unexpected format.") from exc
        if not isinstance(content, str) or not content.strip():
            raise AIError("The model returned an empty answer.")
        return content
