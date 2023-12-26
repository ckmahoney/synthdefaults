# SynthDefaults

These synths were found around the web. You may recognize some of them!

An attempt has been made to include only "musically useful" synthdefs. 
Efforts were also made to normalize the audio levels of a synth using default params.

So requesting any synth with amp `0.1` should return a signal whose overall amplitude/volume are equal.

## Usage

First, clone this repository to your supercolider class path. You can get your classpath by evaluating `Platform.userExtensionDir` or `Platform.classLibraryDir`. 
Clone this repo there, restart supercollider, and it should be auto loaded.


### SynthDefaults.loadSynths()

To make these definitions available to your audio server, call `SynthDefaults.loadSynths`. It returns a Dictionary of (groupName: nSynths). 

### SynthDefaults.opts()
The synthdefs are grouped into some general musical categories, like "organ" or "piano" or "flute". 
To get the options Dictionary after it is loaded, use `SynthDefaults.opts`. 


### SynthDefaults.choose(name: keyword)
To pick one of these at random, use `SynthDefaults.choose`. It will return the **name of the synthdef**. 
Example usage with a Pbind:
```
Pbind(\instrument, Pfunc { SynthDefaults.choose(\flute) }, \note, Pn(55) ).play()
```

The result should be a random flute per note. 

### SynthDefaults.get(name: keyword, n?: integer)
To select a synth more deterministically, use this getter. The "n" value will be modulus the number of available synths. 
So you will always get the same synth per name/n combo. Unless you add more synths! Which would be cool (see below).

### SynthDefaults.any()
Gives you any registered synthdef name.

## About this 

*no guarantees, FOSS, all that schnazz*

synths are super cool.

To be honest I haven't tested this outside of my own SuperCollider environment. 
So if there is some quark or class missing in this code that isn't in your code, please open an issue or pull request to raise awareness.

Also would be super cool if you wanted to open a pull request to like make the synths better.
