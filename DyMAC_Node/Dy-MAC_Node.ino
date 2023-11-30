// LoRa-Node for BT-MAC
#include <SPI.h>
#include <RH_RF95.h>
#include <math.h>
using namespace std;

#define RFM95_CS 10
#define RFM95_RST 9
#define RFM95_INT 2

// different TX frequencies!
#define RF95_TX_FREQ_0 448.8
#define RF95_TX_FREQ_1 449.1
#define RF95_TX_FREQ_2 449.4
#define RF95_TX_FREQ_3 449.7
#define RF95_TX_FREQ_4 450.9
#define RF95_TX_FREQ_5 450.6
#define RF95_TX_FREQ_6 450.3
#define RF95_TX_FREQ_7 450.0
#define RF95_RX_FREQ   480.0

#define RF95_BW 250000
// Change to 7、8、9、10、11、12 OR other SF.
#define RF95_TX_SF_0 7
#define RF95_TX_SF_1 8
#define RF95_TX_SF_2 9
#define RF95_TX_SF_3 10
#define RF95_TX_SF_4 11
#define RF95_TX_SF_5 12
#define RF95_RX_SF   12
// Change to 8、16、32、64... OR others PreambleLength.
#define RF95_PrL 8
// Change to 5、6、7 OR 8 other CodingRate.
#define RF95_CR 7
// Change TxPower from +2 to +20 (For 18, 19 and 20, PA_DAC is enabled).
#define RF95_TxP 10

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Blinky on receipt
#define LED 13


#define debug 0

//channel loading matrix
double CHload[5][6] = {0, 0, 0, 0, 0, 0,
                       0, 0, 0, 0, 0, 0,
                       0, 0, 0, 0, 0, 0,
                       0, 0, 0, 0, 0, 0,
                       0, 0, 0, 0, 0, 0};
//Channel Energy Efficiency Matrix
double CHee[5][6] = {0, 0, 0, 0, 0, 0,
                       0, 0, 0, 0, 0, 0,
                       0, 0, 0, 0, 0, 0,
                       0, 0, 0, 0, 0, 0,
                       0, 0, 0, 0, 0, 0};

//local channel score matrix
double ChannelSelect[5][6] = {0, 0, 0, 0, 0, 0,
                       0, 0, 0, 0, 0, 0,
                       0, 0, 0, 0, 0, 0,
                       0, 0, 0, 0, 0, 0,
                       0, 0, 0, 0, 0, 0};
//channel ordering matrix
double ChannelSort[5][6]={0, 0, 0, 0, 0, 0,
                       0, 0, 0, 0, 0, 0,
                       0, 0, 0, 0, 0, 0,
                       0, 0, 0, 0, 0, 0,
                       0, 0, 0, 0, 0, 0};
//channel count matrix
int ChannelCount[5][6]= {0, 0, 0, 0, 0, 0,
                       0, 0, 0, 0, 0, 0,
                       0, 0, 0, 0, 0, 0,
                       0, 0, 0, 0, 0, 0,
                       0, 0, 0, 0, 0, 0};

//penalty function
double punish(uint8_t a);
void setup() 
{
  pinMode(LED, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  while (!Serial);
  Serial.begin(9600);
  delay(100);

  Serial.println("LoRa MAC Test!");

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

// Setting frequency part
  rf95.setFrequency(RF95_RX_FREQ);
  Serial.print("Set Freq to: "); Serial.println(RF95_RX_FREQ);

// Setting SpreadingFactor part
  rf95.setSpreadingFactor((uint8_t)RF95_RX_SF);
  Serial.print("Set SF to: "); Serial.println(RF95_RX_SF);

// Setting PreambleLength part
  rf95.setPreambleLength((uint16_t )RF95_PrL);
  Serial.print("Set PreambleLength to: "); Serial.println(RF95_PrL);

// Setting CodingRate part
  rf95.setCodingRate4((uint8_t)RF95_CR);
  Serial.print("Set CodingRate to: "); Serial.println(RF95_CR);

// Setting TxPower part
  rf95.setTxPower(RF95_TxP, false);
  Serial.print("Set TxPower to: "); Serial.println(RF95_TxP);

// Setting bandwidth part
  rf95.setSignalBandwidth((long)RF95_BW);
  Serial.print("Set BW to: "); Serial.println(RF95_BW);

Serial.println("---------------------------");

//Resetting Arduino’s Pseudo-Random Number Generator
randomSeed(analogRead(0));
}

int Beaconflag = 0;   //Indicator to determine whether the Beacon is being received,Beaconflag=0 is the receiving beacon state, and Beaconflag=1 is the sending state.
double penalty_factor = 0.0;//penalty factor
double value[3]={0,0,0};
int BestCH,BestSF; //Index used to store the selected channel


//penalty function
double punish(uint8_t a){
    double temp=0;
    temp=log(2)/log(a);
    return temp;
}
void loop()
{
  //waiting beacon to send the message.
  if(Beaconflag == 0)
  {
    
    int select_finish = 0;    //Used to determine whether to select the optimal channel
    //int BestCH,BestSF;     //Optimal channel for storage selection
    
    //Start receive mode to receive data
    if (rf95.available())
    {
      // Should be a message for us now   
      uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
      uint8_t len = sizeof(buf);
      
    
      if (rf95.recv(buf, &len))
      {
        //Determine whether Beacon is a legal format
        if(len != 60)
        {
          Serial.println("---------------------------");
          Serial.println("This is not a legal Beacon !");
          Serial.println("---------------------------");
        }
        //When the sent Beacon is in the correct format, start identifying the Beacon content.
        else 
        {
            int x,y;
            int maxLCH;
            double maxNum;   
            double Beaconload[5][6];
            double Beaconee[5][6];
            int maxten[10];
            int TXcount=0;
            
              for(int i=0; i<5; i++)
              {
                for(int j=0;j<6;j++){
                Beaconload[i][j] = CHload[i][j];  
                Beaconee[i][j] = CHee[i][j];  
                }
                
              }
           //Matrix information printing
           if(debug){
                    //Print load matrix historical values
                    Serial.println("channel load is:");
                    for(int i=0; i<5; i++)
                    {
                      for(int j=0;j<6;j++){
                      Serial.print(CHload[i][j]);Serial.print(" ");
                      if(j==5){
                        Serial.println("");
                      }
                      }
                    }
                    Serial.println("");
                    Serial.println("---------------------------");
              }
            //Get the beacon content and store it in a local array/matrix
            for(int i=0; i<60; i++)
            {
              float value;
              if(buf[i]==42){
                value=1.0;
              }
           
              else if(buf[i]>=48&&buf[i]<=57){
              value=(buf[i]-48)*0.1;
              }
              if(i<30){
               int r=i/6;int c=i%6;
               if(i==0&&debug==1){
                 Serial.println("load msg is:");
               }
               if(debug){
                 Serial.print(value);Serial.print(" ");
                if(i==5 || i==11||i==17||i==23||i==29){
                  Serial.println("");
                }
               }
               Beaconload[r][c]=Beaconload[r][c]*0.50+value*0.50;
              }
              else {
                int r=(i-30)/6;int c=(i-30)%6;
                if(i==30&&debug==1){
                 Serial.println("energy msg is:");
               }
               if(debug){
               Serial.print(value);Serial.print(" ");
               if(i==35||i==41||i==47||i==53||i==59){
                  Serial.println("");
                }
               }
                Beaconee[r][c]=value;
              }
            
            }
            //Print channel count matrix
            if(debug){
            Serial.println("channel count msg is:");
            for(int i=0;i<5;i++){
              for(int j=0;j<6;j++){
                 Serial.print(ChannelCount[i][j]);Serial.print(" ");
                  if(j==5){
                  Serial.println("");
                }
              }
            }
            }

            
            for(int i=0;i<5;i++)
            {
              for(int j=0;j<6;j++){
              
              TXcount = ChannelCount[i][j];
              penalty_factor=punish(TXcount);
              if(TXcount>1){
              
              ChannelSelect[i][j]=penalty_factor*((1-Beaconload[i][j])*0.7+(Beaconee[i][j])*0.3);//Load weight 0.7, energy efficiency weight 0.3
              ChannelSort[i][j]=ChannelSelect[i][j];//
              }
              else{
              ChannelSelect[i][j]=((1-Beaconload[i][j])*0.7+(Beaconee[i][j])*0.3);//Load weight 0.7, energy efficiency weight 0.3
              ChannelSort[i][j]=ChannelSelect[i][j];//
              }
             
              }
            }
            
          
            for(int k=0;k<10;k++)
            {
              maxNum = 0;
              maxten[k] = 0;
              
              for(int i=0;i<5;i++){
                for(int j=0;j<6;j++)
                { 
                  if(ChannelSort[i][j]>maxNum)
                  {
                    maxNum=ChannelSort[i][j];
                    maxten[k]=i*10+j;
                  }
                }
              }
              
              int a=maxten[k]/10;
              int b=maxten[k]%10;
        
              ChannelSort[a][b]=0;

            }
           
            
            //Randomly select one from the array minten containing the 10 largest subscripts.
            
             long randomNum = random(10);
          
            if(randomNum==0)
            {maxLCH = maxten[0];}
            else if(randomNum==1)
            {maxLCH = maxten[1];}
            else if(randomNum==2)
            {maxLCH = maxten[2];}
            else if(randomNum==3)
            {maxLCH = maxten[3];}
             else if(randomNum==4)
            {maxLCH = maxten[4];}
             else if(randomNum==5)
            {maxLCH = maxten[5];}
             else if(randomNum==6)
            {maxLCH = maxten[6];}
             else if(randomNum==7)
            {maxLCH = maxten[7];}
             else if(randomNum==8)
            {maxLCH = maxten[8];}
             else if(randomNum==9)
            {maxLCH = maxten[9];}

            //The SF and CH to be used are determined based on the selected minimum matrix value, and the corresponding transmission parameters are adjusted.
            BestCH=maxLCH/10;
            BestSF=maxLCH%10;
        
            ChannelCount[BestCH][BestSF]++;
            //Set the optimal CH
            switch(BestCH)
            {
              case 0:
                rf95.setFrequency(RF95_TX_FREQ_0);
                break;
              case 1:
                rf95.setFrequency(RF95_TX_FREQ_1);
                break;
              case 2:
                rf95.setFrequency(RF95_TX_FREQ_2);
                break;
              case 3:
                rf95.setFrequency(RF95_TX_FREQ_3);
                break;
              case 4:
                rf95.setFrequency(RF95_TX_FREQ_4);
                break;
              case 5:
                rf95.setFrequency(RF95_TX_FREQ_5);
                break;
              case 6:
                rf95.setFrequency(RF95_TX_FREQ_6);
                break;
              case 7:
                rf95.setFrequency(RF95_TX_FREQ_7);
                break;
              default:
                Serial.println("Setting CHCHCH failed!!!");
            }
            //Set the optimal SF
            switch(BestSF)
            {
              case 0:
                rf95.setSpreadingFactor((uint8_t)RF95_TX_SF_0);
                break;
              case 1:
                rf95.setSpreadingFactor((uint8_t)RF95_TX_SF_1);
                break;
              case 2:
                rf95.setSpreadingFactor((uint8_t)RF95_TX_SF_2);
                break;
              case 3:
                rf95.setSpreadingFactor((uint8_t)RF95_TX_SF_3);
                break;
              case 4:
                rf95.setSpreadingFactor((uint8_t)RF95_TX_SF_4);
                break;
              case 5:
                rf95.setSpreadingFactor((uint8_t)RF95_TX_SF_5);
                break;
              default:
                Serial.println("Setting SFSFSF failed!!!");
            }
            
            select_finish = 1;        //Channel selection process ends

            //Save this time’s load and energy efficiency values
            for(int i=0;i<5;i++){
              for(int j=0;j<6;j++){
              CHload[i][j] = Beaconload[i][j];
              CHee[i][j] = Beaconee[i][j];
              }
            }

//-------------------------------------------------------------------
           //Print channel score matrix
           if(debug){
            Serial.println("ChannelSelect is:");
              for(int i=0;i<5;i++){
              for(int j=0; j<6; j++)
              {
                Serial.print(ChannelSelect[i][j]);Serial.print(" ");
                if(j == 5)
                {Serial.println();}
              }
             }
           }
            Serial.println("---------------------------");
            Serial.print("We are using CH:");Serial.print(BestCH);Serial.print(" and SF:");Serial.println(BestSF);
            Serial.println("---------------------------");
//--------------------------------------------------------------------

        
        
        }//

      }//

            if(select_finish == 1)
            {
                  //After using the receive mode, you must first call setModeIdle() to adjust the status to idle.
                  rf95.setModeIdle();
                  Beaconflag=1;
            }
    }
  }//Beaconflag = 0 

/*--------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
  
   //Sending part
    else if(Beaconflag == 1)
    {
      //setting payload for packet  Part 
    

      char message[24] = "This is a message!!!";
    
    
      rf95.send((uint8_t *)message, sizeof(message));

      //Serial.println("Waiting for packet to complete..."); 
      
      rf95.waitPacketSent();
      Serial.println("Sending completed!");
      //Serial.println("---------------------------");

      //After sending, Beaconflag is set back to 0 and the next wait is started.
      Beaconflag = 0;
      // Setting frequency  and SpreadingFactor 
      rf95.setFrequency(RF95_RX_FREQ);
      rf95.setSpreadingFactor((uint8_t)RF95_RX_SF);
      
      delay(900);
    } 

}


