#!/usr/bin/env python3

import pretty_midi
import json
import sys
import os
import matplotlib.pyplot as plt
import librosa.display

def plot_piano_roll(pm, start_pitch, end_pitch, fs=500):
    # Use librosa's specshow function for displaying the piano roll
    # librosa.display.specshow(pm.get_piano_roll(fs)[start_pitch:end_pitch],
    #                          hop_length=1, sr=fs, x_axis='time', y_axis='cqt_note',
    #                          fmin=pretty_midi.note_number_to_hz(start_pitch))

    roll = pm.get_piano_roll(fs)[start_pitch:end_pitch]

    plt.imshow(roll, aspect='auto', origin='lower', interpolation='nearest')

    plt.ylabel("MIDI note")
    plt.xlabel("Time frame")
    plt.title("Piano roll")
    plt.colorbar(label="Velocity")
    plt.show()
    
# TODO: change lowest_midi later
def midi_to_timeline(midi_path, out_json, lowest_midi=21):
    pm = pretty_midi.PrettyMIDI(midi_path)
    timeline = []
    timestamps = []
    notes = []
    song = []

    for inst_idx, instrument in enumerate(pm.instruments):
        for note in instrument.notes:
            timeline.append({
                'time': round(note.start, 6),
                'event': 'note_on',
                'midi_note': int(note.pitch),
                'key_index': int(note.pitch) - lowest_midi,
                'velocity': int(note.velocity),
                #'instrument': instrument.program if not instrument.is_drum else 'drum'
            })

            timeline.append({
                'time': round(note.end, 6),
                'event': 'note_off',
                'midi_note': int(note.pitch),
                'key_index': int(note.pitch) - lowest_midi,
                'velocity': 0,
                #'instrument': instrument.program if not instrument.is_drum else 'drum'
            })

            timestamps.append(int(round(note.start, 6) * 1000))
            notes.append(int(note.pitch) - lowest_midi)

            song.append("{" + str(notes[-1]) + ", " + str(timestamps[-1]) + ", " + "100" + "},")

    timeline.sort(key=lambda e: (e['time'], 0 if e['event']=='note_on' else 1))

    with open(out_json, 'w') as f:
        json.dump(timeline, f, indent=2)

    out_path = os.path.abspath(out_json)
    print(f'Wrote {len(timeline)} events to {out_path}')

    plot_piano_roll(pm, 21, 108)

    for note in song:
        print(note)

def is_black_key(midi_note):
    return midi_note % 12 in [1, 3, 6, 8, 10]

def is_white_key(midi_note):
    return not is_black_key(midi_note)

# script_path = os.path.abspath(__file__)
# #midi_path = os.path.abspath(os.path.join(script_path, '..', "ETE7-Thomas_the_Tank_Engine_theme.mid"))
# midi_path = r"D:\STM32CubeIDE\workspace_2.0.0\elec391-robo-maestro\ETE7-Thomas_the_Tank_Engine_theme.mid"
# midi_to_timeline(midi_path)

if __name__ == '__main__':
    # if len(sys.argv) < 2:
    #     print('usage: python3 midi_to_notes.py song.mid [out.json]')
    #     sys.exit(1)
    # midi_to_timeline(sys.argv[1], sys.argv[2] if len(sys.argv)>2 else 'timeline.json')
    #midi_path = r"D:\STM32CubeIDE\workspace_2.0.0\elec391-robo-maestro\ETE7-Thomas_the_Tank_Engine_theme.mid"
    midi_path = r"C:\Users\nnog2\Desktop\ELEC Year 4\ELEC391\Thomas The Tank Engine Theme Song.mid"
    out_json = os.path.join(os.path.abspath(__file__), '..', 'timeline.json')
    midi_to_timeline(midi_path, out_json)