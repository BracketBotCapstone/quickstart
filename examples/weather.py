import time

import numpy as np
from transformers import pipeline
from transformers.pipelines.audio_utils import ffmpeg_microphone_live

device = 'cpu'

classifier = pipeline(
    # "audio-classification", model="MIT/ast-finetuned-speech-commands-v2", device=device
    "audio-classification", model="0xb1/wav2vec2-base-finetuned-speech_commands-v0.02", device=device # faster
)
model = "openai/whisper-tiny.en"
# model = "distil-whisper/distil-small.en"
transcriber = pipeline(
    "automatic-speech-recognition", model=model, device=device
)


PERPLEXITY_API_KEY="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"
ELEVEN_LABS_API_KEY = "sk_bf7e6fbfd13cdee351b09adfd2c7a5ceffcfabf89f60fbdb"

from io import BytesIO

import alsaaudio
import sounddevice as sd
import soundfile as sf
from elevenlabs.client import ElevenLabs
from scipy import signal


def set_alsa_volume(volume=75):
    try:
        cards = alsaaudio.cards()
        card_num = None
        for i, card in enumerate(cards):
            if 'Device' in card:
            # if 'UACDemoV10' in card:
                card_num = i
                break
        
        if card_num is None:
            print("Could not find UACDemoV1.0 audio device")
            return
            
        mixer = alsaaudio.Mixer('PCM', cardindex=card_num)
        mixer.setvolume(volume)
        print(f"Set UACDemoV1.0 volume to {volume}%")
    except alsaaudio.ALSAAudioError as e:
        print(f"Error setting volume: {e}")

def speak_text(text, voice="Brian"):
    # Check for special phrases that use pre-recorded audio
    text_normalized = text.lower().strip('-.,!?')
    if text_normalized in ["wow", "uhhhhhhhhh lemme think about that"]:
        # Load and play the corresponding audio file
        filename = "wow.wav" if text_normalized == "wow" else "uhh.wav"
        try:
            data, sample_rate = sf.read(filename, dtype='float32')
            device_name = "pulse"
            # device_name = "UACDemoV1.0"
            device_info = sd.query_devices(device_name, 'output')
            device_id = device_info['index']
            device_sample_rate = device_info['default_samplerate']

            # Resample if needed
            if sample_rate != device_sample_rate:
                number_of_samples = int(round(len(data) * float(device_sample_rate) / sample_rate))
                data = signal.resample(data, number_of_samples)
                sample_rate = device_sample_rate

            sd.play(data, samplerate=sample_rate, device=device_id)
            sd.wait()
            return
        except Exception:
            pass
    """Speaks the given text using ElevenLabs API and plays through audio device."""
    # Initialize ElevenLabs client
    client = ElevenLabs(api_key=ELEVEN_LABS_API_KEY)
    
    # Generate audio from text
    audio = client.generate(
        text=text,
        voice=voice,
        model="eleven_multilingual_v2"
    )
    
    # Set audio device
    device_name = "pulse"  # Name of the USB audio device
    # device_name = "UACDemoV1.0"  # Name of the USB audio device
    device_info = sd.query_devices(device_name, 'output')
    device_id = device_info['index']
    device_sample_rate = device_info['default_samplerate']

    # Prepare audio data
    audio_data = b''.join(audio)
    data, sample_rate = sf.read(BytesIO(audio_data), dtype='float32')

    # Resample if needed
    if sample_rate != device_sample_rate:
        number_of_samples = int(round(len(data) * float(device_sample_rate) / sample_rate))
        data = signal.resample(data, number_of_samples)
        sample_rate = device_sample_rate

    try:
        # Play the audio
        sd.play(data, samplerate=sample_rate, device=device_id)
        sd.wait()
        
        # Save to wav if text was "wow" or "uhh"
        if text_normalized in ["wow", "uhhhhhhhhh lemme think about that"]:
            filename = "wow.wav" if text_normalized == "wow" else "uhh.wav"
            sf.write(filename, data, int(sample_rate))
            
    except sd.PortAudioError as e:
        print(f"Error playing audio: {e}")


import pyaudio


def speak_text_stream(text, voice="Brian"):
    # Check for special phrases that use pre-recorded audio
    text_normalized = text.lower().strip('-.,!?')
    if text_normalized in ["wow", "uhhhhhhhhh lemme think about that"]:
        # Load and play the corresponding audio file
        filename = "wow.wav" if text_normalized == "wow" else "uhh.wav"
        try:
            data, sample_rate = sf.read(filename, dtype='float32')
            
            # Initialize PyAudio
            p = pyaudio.PyAudio()
            
            # Find the device index for "UACDemoV1.0"
            device_id = None
            for i in range(p.get_device_count()):
                device_info = p.get_device_info_by_index(i)
                if device_info['name'] == "UACDemoV1.0":
                    device_id = i
                    break
            
            if device_id is None:
                raise Exception("Audio device UACDemoV1.0 not found")
                
            device_sample_rate = int(p.get_device_info_by_index(device_id)['defaultSampleRate'])

            # Resample if needed
            if sample_rate != device_sample_rate:
                number_of_samples = int(round(len(data) * float(device_sample_rate) / sample_rate))
                data = signal.resample(data, number_of_samples)
                sample_rate = device_sample_rate

            # Open stream
            stream = p.open(format=pyaudio.paFloat32,
                          channels=1,
                          rate=int(sample_rate),
                          output=True,
                          output_device_index=device_id)

            # Play the audio
            stream.write(data.tobytes())
            
            # Cleanup
            stream.stop_stream()
            stream.close()
            p.terminate()
            return
            
        except Exception as e:
            print(f"Error playing audio file: {e}")

    """Streams and plays text-to-speech audio using ElevenLabs API."""
    client = ElevenLabs(api_key=ELEVEN_LABS_API_KEY)
    
    # Initialize PyAudio
    p = pyaudio.PyAudio()
    
    # Find the device index for "UACDemoV1.0"
    device_id = None
    for i in range(p.get_device_count()):
        device_info = p.get_device_info_by_index(i)
        if device_info['name'] == "UACDemoV1.0":
            device_id = i
            break
            
    if device_id is None:
        raise Exception("Audio device UACDemoV1.0 not found")
        
    device_sample_rate = int(p.get_device_info_by_index(device_id)['defaultSampleRate'])

    # Generate streaming audio from text
    audio_stream = client.generate(
        text=text,
        voice=voice,
        model="eleven_multilingual_v2",
        stream=True,
        output_format="pcm_24000"  # Changed to PCM format instead of MP3
    )

    try:
        # Open audio stream
        stream = p.open(format=pyaudio.paFloat32,
                       channels=1,
                       rate=device_sample_rate,
                       output=True,
                       output_device_index=device_id)

        # Process and play audio chunks as they arrive
        chunks_buffer = []
        for chunk in audio_stream:
            if chunk:
                # Convert bytes to numpy array
                audio_data = np.frombuffer(chunk, dtype=np.int16)
                # Normalize to float32 between -1 and 1
                audio_data = audio_data.astype(np.float32) / 32768.0
                
                # Resample if needed (24000 to device_sample_rate)
                if device_sample_rate != 24000:
                    number_of_samples = int(round(len(audio_data) * float(device_sample_rate) / 24000))
                    audio_data = signal.resample(audio_data, number_of_samples)

                chunks_buffer.append(audio_data)

                # Play after accumulating 25 chunks
                if len(chunks_buffer) >= 25:
                    # Concatenate chunks
                    combined_audio = np.concatenate(chunks_buffer)
                    
                    # Calculate and print duration in seconds
                    duration = len(combined_audio) / device_sample_rate
                    print(f"Combined chunks duration: {duration:.2f} seconds")
                    
                    # Play the combined audio
                    stream.write(combined_audio.tobytes())
                    
                    # Clear buffer
                    chunks_buffer = []

        # Play any remaining chunks
        if chunks_buffer:
            combined_audio = np.concatenate(chunks_buffer)
            stream.write(combined_audio.tobytes())

        # Save to wav if text was "wow" or "uhh"
        if text_normalized in ["wow", "uhhhhhhhhh lemme think about that"]:
            filename = "wow.wav" if text_normalized == "wow" else "uhh.wav"
            sf.write(filename, data, int(sample_rate))

        # Cleanup
        stream.stop_stream()
        stream.close()
        p.terminate()
            
    except Exception as e:
        print(f"Error playing audio: {e}")
        # Cleanup in case of error
        if 'stream' in locals():
            stream.stop_stream()
            stream.close()
        if 'p' in locals():
            p.terminate()


def wait_for_wow(
    prob_threshold=0.5,
    debug=True,
):
    sampling_rate = classifier.feature_extractor.sampling_rate

    mic = ffmpeg_microphone_live(
        sampling_rate=sampling_rate,
        chunk_length_s=0.75,
        stream_chunk_s=0.5,
    )

    wake_word = 'wow'
    if wake_word not in classifier.model.config.label2id.keys():
        raise ValueError(
            f"Wake word {wake_word} not in set of valid class labels, pick a wake word in the set {classifier.model.config.label2id.keys()}."
        )


    print("Listening for wake word...")
    for prediction in classifier(mic):
        prediction = prediction[0]
        if debug:
            print(prediction)
        if prediction["label"] == wake_word:
            if prediction["score"] > prob_threshold:
                mic.close()
                return True


def transcribe(chunk_length_s=5.0, stream_chunk_s=2.0):
    sampling_rate = transcriber.feature_extractor.sampling_rate
    # sampling_rate = classifier.feature_extractor.sampling_rate


    print("Start speaking...")
    try:
        mic = None
        while True:
            if mic:
                mic.close()
            wait_for_wow()
            speak_text('WOW!!')
            print('WOW')
            weather_said = False
            start_time = time.time()
            mic = ffmpeg_microphone_live(
                sampling_rate=sampling_rate,
                chunk_length_s=chunk_length_s,
                stream_chunk_s=stream_chunk_s,
            )
            while not weather_said and (time.time()-start_time < 15):
                for item in transcriber(mic, generate_kwargs={"max_new_tokens": 128}):
                    # sys.stdout.write("\033[K")
                    current_text = item["text"].lower()
                    print(current_text)

                    # Check if "weather" appears or disappears between transcriptions
                    if "weather" in current_text:
                        speak_text("fine... I'll get the forecast I guess...")
                        on_weather_change()
                        weather_said = True
                        break
                        # print(current_text)
                    
                    if current_text.endswith('?'):
                        speak_text("-uhhhhhhhhh lemme think about that...")
                        answer = ask_perplexity(current_text)
                        print(answer)
                        speak_text(answer)
                        weather_said = True
                        break
                    
                    
                    if not item["partial"][0]:
                        print()  # Move to next line when chunk is complete
                        break

            if not weather_said:
                speak_text("Bitch what did you say WOW for!")
            
    except KeyboardInterrupt:
        print("\nStopped by user")


def format_weather_info(data):
    """Format weather data into a human-readable string."""
    current = data['current']
    
    # Extract relevant information
    temp = current['temp']
    feels_like = current['feels_like']
    weather = current['weather'][0]['description']
    humidity = current['humidity']
    wind_speed = current['wind_speed']
    
    # Construct weather message
    message = "\nCurrent weather conditions:"
    message += f"\nTemperature: {temp}°C (feels like {feels_like}°C)"
    message += f"\nConditions: {weather}"
    message += f"\nHumidity: {humidity}%"
    message += f"\nWind Speed: {wind_speed} m/s\n"
    
    return message

def on_weather_change():

    try:
        weather = weather_perplexity()
        print(weather)
        speak_text(weather)
        
    except Exception as e:
        print(f"\nError getting weather: {e!s}\n")

from openai import OpenAI


def weather_perplexity():
    client = OpenAI(api_key=PERPLEXITY_API_KEY, base_url="https://api.perplexity.ai")
    
    messages = [
        {
            "role": "system",
            "content": (
                "You are a weather assistant. Give a simple weather report using only "
                "common words that can be easily spoken and heard. Use exactly one "
                "sentence. Do not include dates and use the word degrees instead of "
                "the degrees symbol."
            ),
        },
        {
            "role": "user", 
            "content": (
                "Tell me the current temperature and weather conditions in Waterloo, "
                "Ontario in one simple sentence."
            ),
        },
    ]

    try:
        response = client.chat.completions.create(
            model="llama-3.1-sonar-large-128k-online",
            messages=messages,
        )
        
        # Extract the weather information from the response
        weather_info = response.choices[0].message.content
        return weather_info
    
    except Exception as e:
        return f"Error getting weather information: {e!s}"
    

def ask_perplexity(question):
    client = OpenAI(api_key=PERPLEXITY_API_KEY, base_url="https://api.perplexity.ai")
    messages = [
        {
            "role": "system", 
            "content": (
                "You're a chill chatbot who keeps things super casual and brief. Just give me one quick sentence "
                "using everyday language, like you're texting a friend. I'm going to just ask stupid questions, try to give me chill answers"
            ),
        },
        {
            "role": "user",
            "content": question,
        },
    ]

    try:
        response = client.chat.completions.create(
            model="llama-3.1-sonar-large-128k-online", 
            messages=messages,
        )
        
        return response.choices[0].message.content
    
    except Exception as e:
        return f"Error getting response: {e!s}"


# speak_text_stream("hey my name is beff jezos I scam retards")
# speak_text("hey my name is beff jezos I scam retards")
set_alsa_volume(100)
transcribe()