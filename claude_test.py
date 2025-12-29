import os
from anthropic import Anthropic, HUMAN_PROMPT, AI_PROMPT
api_key = os.environ.get("ANTHROPIC_API_KEY")
if not api_key:
    print("ANTHROPIC_API_KEY not set. Set it with: setx ANTHROPIC_API_KEY \"YOUR_KEY\"")
    raise SystemExit(1)
client = Anthropic(api_key=api_key)
prompt = HUMAN_PROMPT + "Write a simple Arduino sketch that blinks an LED on pin 13." + AI_PROMPT
resp = client.completions.create(model="claude-2.1", prompt=prompt, max_tokens_to_sample=300)
print(resp.completion)
