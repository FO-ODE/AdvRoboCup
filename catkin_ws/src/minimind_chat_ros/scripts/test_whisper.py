import whisper
import sounddevice as sd
import numpy as np
import wave

# Recording parameters
samplerate = 16000
duration = 5
filename = "test.wav"

print("🎤 Recording...")
recording = sd.rec(int(duration * samplerate), samplerate=samplerate, channels=1, dtype='int16')
sd.wait()

# Save audio
with wave.open(filename, 'wb') as wf:
    wf.setnchannels(1)
    wf.setsampwidth(2)
    wf.setframerate(samplerate)
    wf.writeframes(recording.tobytes())

# Whisper recognition
model = whisper.load_model("base")
result = model.transcribe(filename)
print("🗣", result["text"])
