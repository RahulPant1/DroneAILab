"""Local LLM assistant (Ollama over HTTP only). The model has no tools, shell or file access:
it only ever sees a compact JSON summary of simulator runs, and its suggestions are validated
and applied only when the user clicks Apply."""
