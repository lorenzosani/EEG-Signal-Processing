/*  
 * Reads in EEG data through the microphone input of the 
 * computer, then displays the signal in time, its frequency
 * components, and then averages of frequencies that estimate
 * the concentration of different brain waves.
 *
 * For reference, the frequency bars are ordered/classified as
 * follows:
 *
 * 1 - blue -------- delta
 * 2 - blue/purple - theta
 * 3 - purple ------ alpha 
 * 4 - purple/red -- low beta
 * 5 - dark red ---- mid beta
 * 6 - red --------- high beta
 * 
 * This sketch will measure all brain waves, from 0 - 30 Hz. It does
 * best, however, measuring alpha waves across the occipital lobe.
 * To view this function, play the program, click the window to
 * make sure its in "focus", and hit the "a" key to bandpass the alpha
 * waves only. The alpha wave bar is the 3rd one (purple), and should
 * increase in height by 2-3x when you close your eyes and relax
 * (you'll see it for a second or two after you open your eyes, before it
 * averages back down).
 * /

/* One issue: when taking the FFT of the data, it seems as if
the frequency bands have a bandwidth of 1.33 instead of 1, as 
60Hz noise peaks out at band 45. This is worked around by using
the scaleFreq parameter, which is used frequently. */
import processing.serial.*;
import ddf.minim.*;
import ddf.minim.signals.*;
import ddf.minim.analysis.*;
import ddf.minim.effects.*;
import javax.sound.sampled.*;

//Important constants that may need to be changed.
float timeScale = 50; //scales the amplitude of time-domain data, can be changed
static float normalScale = 50;
static float alphaScale = 100;
static int freqAvgScale = 50; //does same for averages of frequency data
static int alphaCenter = 10;
static int alphaBandwidth = 2; //really bandwidth divided by 2
static int betaCenter = 25;
static int betaBandwidth = 10; //really bandwidth divided by 2

//Upper bound in hertz of different wave-types
static int delta = 3;
static int theta = 8;
static int alpha = 12;
static int lo_beta = 15;
static int mid_beta = 22;
static int hi_beta = 38;

//Variables used to store data functions/effects.
Minim minim;
Serial myPort;

// This scales the buffer size, needs to be a power of two
int SCALE = 1;
int BUFFER = 32768/SCALE;
  
final int timeLength = BUFFER;
float[] timeSignal = new float[timeLength];
FFT fft;
NotchFilter notch;
LowPassSP lpSP;
LowPassFS lpFS;
BandPass betaFilter;
BandPass alphaFilter;

//Constants mainly used for scaling the data to readable sizes.
int windowLength = 840;
int windowHeight = 500;
int FFTheight;
float scaling[] = {.00202,.002449/2,.0075502/2,.00589,.008864,.01777};
int FFTrectWidth = 10;
float scaleFreq = 1.343f;
float timeDomainAverage = 5;

//Variables used to handle bad data
int cutoffHeight = 200; //frequency height to throw out "bad data" for averaging after
float absoluteCutoff = 1.5;
boolean absoluteBadDataFlag; //data that is bad because it's way too far out of our desired range --
                             // ex: shaking your head for a second
boolean averageBadDataFlag;  //data that's bad because it spikes too far outside of the average for 
                             //that second -- 
                             // ex: blinking your eyes for a split second

//Constants used to create a running average of the data.
float[][] averages;
int averageLength = 60; //number of milliseconds to average out
int averageBins = 6; //we have 6 types of brain waves
int counter = 0;

// There are four ways of reading the data into the system:
final int INPUT_FROM_RAW_AUDIO = 0;
final int INPUT_FROM_BUFFERED_AUDIO = 1;
final int INPUT_FROM_SERIAL_PORT = 2;
final int INPUT_FROM_TEST_DATA = 3;

int readDataFrom = INPUT_FROM_BUFFERED_AUDIO;

boolean debugTraces = false;    // swtich off the 'averageBadData flags', which slows things down a lot
final int NUM_CHANNELS = 1;
AudioInput in;


void setup()
{
  // For serial input, you need an Arduino to feed the data back to processing.
  if (readDataFrom == INPUT_FROM_SERIAL_PORT) {
    int serialIndex = 0;
    myPort = new Serial(this, Serial.list()[serialIndex], 9600);
  }
  
  //initialize array of averages for running average calculation
  averages = new float[averageBins][averageLength];
  for (int i = 0; i < averageBins; i++){
    for (int j = 0; j < averageLength; j++){
      averages[i][j] = 0;
    }
  }
  
  //set some drawing parameters
  FFTheight = windowHeight - 200;
  size(840, 500, P2D);
  
  //initialize minim, as well as some filters
  minim = new Minim(this);
  if (debugTraces) {
    minim.debugOn();
  }
  notch = new NotchFilter(60, 10, BUFFER);
  lpSP = new LowPassSP(40, BUFFER);
  lpFS = new LowPassFS(60, BUFFER);
  betaFilter = new BandPass(betaCenter/scaleFreq,betaBandwidth/scaleFreq,BUFFER);
  alphaFilter = new BandPass(alphaCenter/scaleFreq,alphaBandwidth/scaleFreq,BUFFER);
  
  // initialize values in array that will be used for input
  for (int i = 0; i < timeLength; i++){
    timeSignal[i] = 0;
  }

  Mixer.Info[] mixerInfo;
  mixerInfo = AudioSystem.getMixerInfo();
  println("Available mixers:");
  for(int i = 0; i < mixerInfo.length; i++)  {
    println(i +" : " + mixerInfo[i].getName());
  } 
  
  int mixerID = 0;

  Mixer mixer = AudioSystem.getMixer(mixerInfo[mixerID]);
  minim.setInputMixer(mixer);
  in = minim.getLineIn(Minim.MONO, BUFFER, 44100/SCALE);
  //initialize FFT
  fft = new FFT(BUFFER, BUFFER);
  fft.window(FFT.HAMMING);
  rectMode(CORNERS);
  println("FFT Bandwidth: "+fft.getBandWidth());
  
  if (in == null) {
    exit();
  }
}

// MAIN FUNCTION, called every frame
void draw()
{
  // Gets the audio data
  timeSignal = in.mix.toArray();
  fft.forward(in.mix);
  
  // Handles any "artifacts" we may pick up while recording the data
  absoluteBadDataFlag = false;
  averageBadDataFlag = false;

  // Sets the appearance of the UI
  background(0); //make sure the background color is black
  stroke(255);   //and that time data is drawn in white
  line(0,100,windowLength,100); //line separating time and frequency data
  
  // Draws the frequency line at the top and frequency bars in the middle
  drawSignalData();
  
  //check for spikes relative to other data
  for (int i = 0; i < windowLength - 1; i++){
    if (abs(getData(i+1, windowLength))  > timeDomainAverage*4)
      averageBadDataFlag = true;
  }
  
  // Shows the info text
  displayText();
  
  // Draws the frequency averages on the bottom
  displayFreqAverages();
  
  counter++;
  }


float getData(int index, int ofTotal) {
  int remappedIndex;
 
 switch(readDataFrom) {
    case INPUT_FROM_RAW_AUDIO:
       remappedIndex = round((index * in.bufferSize())/ofTotal);
       return in.left.get(remappedIndex);
       
    case INPUT_FROM_BUFFERED_AUDIO:
    case INPUT_FROM_TEST_DATA:
    case INPUT_FROM_SERIAL_PORT:
       remappedIndex = round((index * timeLength)/ofTotal);
       return timeSignal[remappedIndex];
    }
    return 0;
}


void keyPressed(){
  if (key == 'w'){
    fft.window(FFT.HAMMING);
  }
  if (key == 'e'){
    fft.window(FFT.NONE);
  }
}

//Shifts all elements in an array n times left, resulting in the 
//[0-n] elements being pushed off, and the last numShift elements
//becoming zero. Does this for all data channels.
public void shiftNtimes(float[] array, int n){
  int timesShifted = 0;
  while (timesShifted < n){
    //for (int i = 0; i < NUM_CHANNELS; i++){
      for (int j = 0; j < timeLength - 1; j++){
        array[j] = array[j + 1];
      }
      array[(int)timeLength - 1] = 0;
      timesShifted++;
   // }
  }
}

//Draw the signal in time and frequency.
void drawSignalData(){
  timeDomainAverage = 0;
  for(int i = 0; i < windowLength - 1; i++){
      float dataValue = getData(i, windowLength);
      float nextValue = getData(i+1, windowLength);
      
      stroke(255,255,255);
      //data that fills our window is normalized to +-1, so we want to throw out
      //sets that have data that exceed this by the factor absoluteCutoff
      if (abs(dataValue) * timeScale/normalScale > .95){
      //if (abs(in.left.get(i*round(in.bufferSize()/windowLength)))*timeScale/normalScale > .95){
          absoluteBadDataFlag = true;
          fill(250,250,250);
          stroke(150,150,150);
        }
      //Draw the time domain signal.
      line(i, 50 + dataValue*timeScale, 
           i+1, 50 + nextValue*timeScale);
           
      timeDomainAverage += abs(dataValue);
      //Draw un-averaged frequency bands of signal.
      if (i < (windowLength - 1)/2){
        //set colors for each type of brain wave
          if (i <= round(delta/scaleFreq)){ 
            fill(0,0,250);        //delta
            stroke(25,0,225);
          }
          if (i > round(delta/scaleFreq) && i <= round(theta/scaleFreq)){
            fill(50,0,200);       //theta
            stroke(75,0,175);
          }
          if (i > round(theta/scaleFreq) && i <= round(alpha/scaleFreq)){  
            fill(100,0,150);      //alpha
            stroke(125,0,125);
          }
          if (i > round(alpha/scaleFreq) && i <= round(lo_beta/scaleFreq)){
            fill(150,0,100);      //low beta
            stroke(175,0,75);
          }
          if (i > round(lo_beta/scaleFreq) && i <= round(mid_beta/scaleFreq)){ 
            fill(200,0,50);       //midrange beta
            stroke(225,0,25);
          }
          if (i > round(mid_beta/scaleFreq) && i <= round(hi_beta/scaleFreq)){ 
            fill(250,0,0);        //high beta
            stroke(255,0,10);
          }
          if (i > round(hi_beta/scaleFreq)){
            fill(240,240,240);    //rest of stuff, mainly noise
            stroke(200,200,200);
          }
          if (i == round(50/scaleFreq) || i == round(100/scaleFreq)){
            fill(200,200,200);    //color 50 Hz a different tone of grey,
            stroke(150,150,150);  //to see how much noise is in data
          }
        //draw the actual frequency bars
        rect(FFTrectWidth*i, FFTheight, FFTrectWidth*(i+1), FFTheight - (fft.getFreq(i)*4));
      }
    }
  //divide the average by how many time points we have
  timeDomainAverage = timeDomainAverage / (windowLength - 1);
}

//Give user textual information on data being thrown out and filters we have active.
void displayText(){
  //show user when data is being thrown out
  text("absoluteBadDataFlag = " + absoluteBadDataFlag, windowLength - 200, 120);
  if (absoluteBadDataFlag == true && debugTraces)
  {
    println("absoluteBadDataFlag = " + absoluteBadDataFlag);
    println(counter);
  }
  text("averageBadDataFlag = " + averageBadDataFlag, windowLength - 200, 140);
  if (averageBadDataFlag == true && debugTraces)
  {
    println("averageBadDataFlag = " + averageBadDataFlag);
    println(counter);
  }

  //and when a filter is being applied to the data
  text("alpha filter is " + in.hasEffect(alphaFilter),
    windowLength - 200, 160);
  text("beta filter is " + in.hasEffect(betaFilter),
    windowLength - 200, 180);
}

//Compute and display averages for each brain wave for the past ~5 seconds.
void displayFreqAverages(){
  //show averages of alpha, beta, etc. waves
  for (int i=0; i < 6; i++){
    float avg = 0; //raw data for amplitude of section of frequency
    int lowFreq = 0;
    int hiFreq = 0;
  
    //Set custom frequency ranges to be averaged. 
    if(i == 0){
      lowFreq = 0;
      hiFreq = delta;
      fill(0,0,250);
      stroke(25,0,225);
    }
    if(i == 1){
      lowFreq = delta+1;
      hiFreq = theta;
      fill(50,0,200);
      stroke(75,0,175);
    }
    if(i == 2){
      lowFreq = theta+1;
      hiFreq = alpha;
      fill(100,0,150);
      stroke(125,0,125);
    }
    if(i == 3){
      lowFreq = alpha+1;
      hiFreq = lo_beta;
      fill(150,0,100);
      stroke(175,0,75);
    }
    if(i == 4){
      lowFreq = lo_beta+1;
      hiFreq = mid_beta;
      fill(200,0,50);
      stroke(225,0,25);
    }
    if(i == 5){
      lowFreq = mid_beta+1;
      hiFreq = hi_beta;
      fill(250,0,0);
      stroke(255,0,10);
    }
    
    //Convert frequencies we want to the actual FFT bands. Because of our
    //FFT parameters, these happen to be equal (each band has a 1 Hz width).
    int lowBound = fft.freqToIndex(lowFreq);
    int hiBound = fft.freqToIndex(hiFreq);
    
    //Scale the band number, because of the issue outlined at very beginning of
    //program.
    lowBound = round(lowBound/scaleFreq);
    hiBound = round(hiBound/scaleFreq);
    
    //get average for frequencies in range
    for (int j = lowBound; j <= hiBound; j++){
      avg += fft.getBand(j);
      }
    avg /= (hiBound - lowBound + 1);
    
    // Scale the bars so that it fits our window a little better.
    for (int k = 0; k < 6; k++)
    {
      if (i == k)
      {
        avg *= scaling[i]*freqAvgScale;
      }
    }
    
    //update our array for the moving average (only if our data is "good")
    if (absoluteBadDataFlag == false && averageBadDataFlag == false){
      averages[i][counter%averageLength] = avg;
    }
    
    //calculate the running average for each frequency range
    float sum = 0;
    for (int k = 0; k < averageLength; k++){
      sum += averages[i][k];
    }
    sum = sum / averageLength;
      
    
    //draw averaged/smoothed frequency ranges
    rect(i*windowLength/6, windowHeight-10, (i+1)*windowLength/6, windowHeight-10-(sum*2));
    //rect(i*36, 600, (i+1)*36, 600 - sum);
  }
}

// always close Minim audio classes when you are done with them
void stop()
{
  in.close();
  minim.stop();
  super.stop();
}
