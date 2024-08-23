//brink led by specific serial input
//Serial receive: [float,float,float,float]
//action: if sum of input is greater than 1, blink led for 1 second
void setup(){
    Serial.begin(115200);
    pinMode(13, OUTPUT);

}

void loop(){
    if(Serial.available()>0){
        String input = Serial.readString();
        float sum = 0;
        int comma = 0;
        for(int i=0; i<input.length(); i++){
            if(input[i]==','){
                comma++;
            }
        }
        if(comma==3){
            float f1 = input.substring(0, input.indexOf(',')).toFloat();
            input = input.substring(input.indexOf(',')+1);
            float f2 = input.substring(0, input.indexOf(',')).toFloat();
            input = input.substring(input.indexOf(',')+1);
            float f3 = input.substring(0, input.indexOf(',')).toFloat();
            input = input.substring(input.indexOf(',')+1);
            float f4 = input.toFloat();
            sum = f1+f2+f3+f4;
        }
        if(sum>1){
            digitalWrite(13, HIGH);
            delay(1000);
            digitalWrite(13, LOW);
        }
    }
    
    
}

