# SynthDefaults

These synths were found around the web. You may recognize some of them!

An attempt has been made to include only "musically useful" synthdefs. 
Efforts were also made to normalize the audio levels of a synth using default params.

So requesting any synth with amp `0.1` should return a signal whose overall amplitude/volume are equal.

## Usage

First, clone `SynthDefaults.sc` to your supercolider class path. You can get your classpath by evaluating `Platform.userExtensionDir` or `Platform.classLibraryDir`. 
Clone this repo there, restart supercollider, and it should be auto loaded.

To make these definitions available to your audio server, call `SynthDefaults.loadSynths`. It returns a Dictionary of (groupName: nSynths). 
To get the options Dictionary after it is loaded, use `SynthDefaults.opts`. 

The synthdefs are grouped into some general musical categories, like "organ" or "piano" or "flute". 

To pick one of these at random, use `SynthDefaults.choose`. It will return the **name of the synthdef**. 
Example usage with a Pbind:
```
Pbind(\instrument, Pfunc { SynthDefaults.choose(\flute) }, \note, Pn(55) ).play()
```

The result should be a random flute per note. 
