//**************************************************************************
// ﾌｧｲﾙ内容     「rmc_r4m1_advnaced_ver1」モータドライブ基板Advanced・アナログセンサ基板TypeS プログラム
// Copyright    ジャパンマイコンカーラリー実行委員会
// ライセンス   This software is released under the MIT License.
//              http://opensource.org/licenses/mit-license.php
//**************************************************************************

//======================================
// インクルード
//======================================
#include <SPI.h>                        // microSDとSPI通信するためのライブラリ
#include <SD.h>                         // microSD読み書きライブラリ
#include <EEPROM.h>                     // EEP-ROM(データフラッシュメモリ)読み書き
#include "FspTimer.h"                   // fsp_timerを使用するためのライブラリ
#include "mcr_gpt_lib.h"                // GPTを使用するためのライブラリ
#include "mcr_sw_lib.h"                 // スイッチのキーリピートなどを使用するためのライブラリ
#include "mcr_ad_lib.h"                 // A/D変換を使用するためのライブラリ
#include "mcr_sp_lib.h"                 // 圧電スピーカーを使用するためのライブラリ
//#include <MGLCD.h>                      // 出典；しなぷすのハード製作記　https://synapse.kyoto/lib/MGLCD/page001.html

//======================================
// シンボル定義
//======================================
const char *C_DATE = __DATE__;          // microSDのファイル日付用 コンパイルした日付
const char *C_TIME = __TIME__;          //                         コンパイルした時間

//#define DEBUG_PRINT //printfするとき有効化する

// ログ処理関係
#define LOG_MAX       (100)             // LOG_MAX × 10ms分のログをRAMに貯める ログが飛ぶときは増やしてください ただし、メモリの上限を超えるとエラーになります
#define LOG_NON       (0)               // ログの状態 何もしない
#define LOG_INIT      (1)               //            初期化
#define LOG_INIT_END  (2)               //            初期化
#define LOG_WRITE     (3)               //            書き込み
#define LOG_END       (4)               //            書き込み終了

#define MOTOR_CYCLE   47999             // 1ms/{1/(HOCO/1)} = 0.001 /{1/(48e6/1)} = 48000
#define FREE          1                 // モータモード　フリー
#define BRAKE         0                 // モータモード　ブレーキ

#define NN 2                            // 音階をNNターブ上げる
#define DO (523*NN)
#define RE (587*NN)
#define MI (659*NN)
#define FA (698*NN)
#define SO (784*NN)
#define RA (880*NN)
#define SHI (988*NN)
#define DO2 (1047*NN)

/* TSL1401CL */
#define	TSL1401_SI_HIGH	  R_PORT3->PODR_b.PODR4 = 1	/* D8 */
#define	TSL1401_SI_LOW	  R_PORT3->PODR_b.PODR4 = 0	/* D8 */
#define	TSL1401_CLK_HIGH	R_PORT3->PODR_b.PODR3 = 1	/* D9 */
#define	TSL1401_CLK_LOW	  R_PORT3->PODR_b.PODR3 = 0	/* D9 */


#define 	TSL1401_LineStart 	40		/* カメラで見る範囲(通常モード) */
#define 	TSL1401_LineStop  	87
#define 	TSL1401_LineStartSaka 	50		/* カメラで見る範囲(坂モード) */
#define 	TSL1401_LineStopSaka  	77
#define 	TSL1401_LineStartNonR 	40		/* カメラで見る範囲(右無視) */
#define 	TSL1401_LineStopNonR  	60
#define 	TSL1401_LineStartNonL 	68		/* カメラで見る範囲(左無視) */
#define 	TSL1401_LineStopNonL  	87


//======================================
// プロトタイプ宣言
//======================================
void AGTCallback(timer_callback_args_t __attribute((unused)) * p_args);
unsigned char sensor_inp( void );
unsigned char center_inp( void );
unsigned char startbar_get( void );
unsigned char dipsw_get( void );
unsigned char pushsw_get( void );
unsigned char get_cn4( void );
void led_out( unsigned char led );
void led_out_m( unsigned char led );
void motor_r( int accele_l, int accele_r );
void motor_f( int accele_l, int accele_r );
void motor_mode_r( int mode_l, int mode_r );
void motor_mode_f( int mode_l, int mode_r );
void motor_st( int pwm );
int check_crossline( void );
int check_rightline( void );
int check_leftline_forC( void );
int check_rightline_forC( void );
int check_leftline( void );
int getServoAngle( void );
int getSakaAngle( void );
void servoControl( void );
void servoControl2( void );
int diff( int pwm );

unsigned long convertBCD_CharToLong( unsigned char hex );
void set_microSD_dateTime( uint16_t* date, uint16_t* time );

void tsl1401(void);
void ImageCapture(int,int);
void binarization(int,int);
void WhiteLineWide(int,int);

//======================================
// グローバル変数の宣言
//======================================
FspTimer fsp_timer;                     // 割り込み関係
mcr_ad ad;                              // A/D変換用
mcr_sp sp;                              // 圧電スピーカー用
mcr_sw sw4,sw3,sw2,sw1, sw0;            // スイッチSW4～SW0用

volatile int pattern = 0;                   // マイコンカー動作パターン
volatile unsigned long cnt_start = 0;       // スタートしてから1msごとに＋１
volatile unsigned long cnt1 = 0;            // タイマ用
volatile unsigned long cnt_check_sen = 0;   // タイマ用
volatile unsigned long cnt_check_enc = 0;   // タイマ用
volatile int f_setup_end = 0;               // setupが終了したら1になる
volatile int angle0_ad = 0;                 // 0度のときのgetServoAngle A/D値

volatile unsigned long   ul_cnt_straight_time_1ms = 0;/* 直線とカーブのカウント用   */
volatile unsigned long   ul_cnt_curve_time_1ms = 0;   // カーブ加速用のカウント用

volatile int saka_cnt = 0; //坂の回数

//char _buf[128];                         // LCD文字列作業用
char _fn_buf[16];                       // ファイルネーム文字列作業用

// エンコーダ関連
volatile long enc_total;                // 積算値(距離)保存用
volatile long enc_kizyun;               // 基準
volatile int enc;                       // 10ms毎の最新値

//  ステアリングモータ関連
volatile int pwm_trace;                 // 中心線トレーズ時のmotor_st PWM値
volatile int pwm_kakudo;                // 角度制御のときのmotor_st PWM値
volatile int set_angle;                 // 指示角度

volatile int fmotor,rmotor;

volatile char	Cu_flag = 0;			//0 = 直線, 1 = カーブ

// 坂道検出用
volatile unsigned long cnt_up;          // 上り坂 しきい値以上の連続時間

//TSL1401
volatile int ImageData[130];			      // カメラの値				
volatile int BinarizationData[130];	    // ２値化	
			
volatile int tsl1401_Max = 0,tsl1401_Max2 = 0,tsl1401_Min,tsl1401_Min2,tsl1401_Ave;	//カメラ読み取り最大値、最小値、平均値
volatile int tsl1401_WB_ave = 0;
volatile unsigned int 	tsl1401_Rsensor;				/* ラインの右端 */
volatile unsigned int 	tsl1401_Lsensor;				/* ラインの左端 */
volatile unsigned int 	tsl1401_Wide = 0;				/* ラインの幅 */
volatile int		        tsl1401_Center = 0;				/* ラインの重心 */
volatile int		        tsl1401_Center_lasttime = 64;		/* 前回のラインの重心 */
volatile int		        tsl1401_Center64 = 0;				/* ラインの重心 */
volatile int            tsl1401_White;					/* 白の数	*/
volatile int		        tsl1401_mode;				/* 0 = 通常 1 = 坂 2 = 右無視 3 = 左無視	*/

// 内輪差値計算用　各マイコンカーに合わせて再計算して下さい
const int revolution_difference[] = {   // 角度から内輪、外輪回転差計算
    100, 98, 97, 95, 94,
    92, 91, 89, 88, 87,
    85, 84, 82, 81, 80,
    78, 77, 75, 74, 73,
    71, 70, 69, 68, 66,
    65, 64, 62, 61, 60,
    58, 57, 56, 54, 53,
    52, 50, 49, 47, 46,
    45, 43, 42, 40, 39,
    37
};

// microSD関連変数
ArduinoSPI SPI(MISO1, MOSI1, SCK1, FORCE_SPI1_MODE); // RMC-RA4M1のmicroSD用SPIを選択(SPIの各端子はpins_arduino.hで定義)
File microSD;                           // microSDのファイルアクセス用
int f_err_sd;                           // 1:mciroSDエラー 0:エラーなし
int log_pattern;                        // ログパターン
int log_st;                             // ログの状態
int log_write;                          // 構造体への書き込み位置
int log_read;                           // 構造体からの読み込み位置
typedef struct {
    uint16_t time;                      // 時間
    uint16_t ptn;                       // パターン
    int16_t tsl1401_Center;             // カメラセンター値
    int16_t tsl1401_Wide;               // カメラ ライン幅値
    int16_t angle;                      // 角度
    int16_t encoder;                    // エンコーダ
    char lf;                            // モータ左前
    char rf;                            // モータ右前
    char lr;                            // モータ左後
    char rr;                            // モータ右後
    char st;                            // モータステアリング
    int16_t saka_ad;                    // 坂AD
    char f_updown;                      // 上り坂、下り坂フラグ
} log_t;
log_t log_buff[LOG_MAX];                // ログ保存用構造体
log_t log_raw;                          // ログ現在の値


// 液晶関係
//MGLCD_AQM1248A_SoftwareSPI MGLCD(MGLCD_SpiPin4( 12, 11, 10, 9),0); // (SCK,MOSI,CS,DI),Wait
//volatile unsigned long cnt_lcd;         // LCD処理で使用
//volatile int f_lcd;                     // 1msごとに1になる

// 各種センサ
volatile int ang;                       // 角度
volatile int ang_old;                       // 角度
volatile int saka;                      // 坂センサ（アナログ）
volatile char f_saka_up;                // 坂アップフラグ 1:UP
volatile char f_saka_down;              // 坂アップフラグ 1:DOWN
volatile float vbat;                    // 電圧



//--------------------------------------------------------------------------------------
//パラメータ
/* 最大走行時間 (0.01秒)  1200 = 12s   */
#define 	MAXTIME 			500 //1100	 

volatile uint16_t df_debug = 0;             // デバッグモード

volatile float df_p = 7.00;                    // PD制御のP値
volatile float df_d = 0.500;                    // PD制御のD値

volatile int		    i_TOPSPEED	=		50;		//直線

#define		MOTOR_OUT_BASE			100		//カーブ用　外側モーター用パラメーター

//前半
volatile int			i_SPEED_DOWN	=		400;//5		//角度によりi_TOPSPEEDを減速 カーブ前半 8 6
volatile int			i_MOTOR_out_R	=	 	100;//1		//外側モーター用パラメーター 1	-2
volatile int			i_MOTOR_in_F	=		300;//4		//内側モーター用パラメーター 	2 	1
volatile int			i_MOTOR_in_R	=		100;//-2		//内側モーター用パラメーター -2	-3

//後半
#define		Cu_N_time			200	//Cu_N_time ms カーブを走行すると後半になる 

volatile int			i_SPEED_DOWN_N=		500;//7		//角度によりi_TOPSPEEDを減速  カーブ後半 11 10
volatile int			i_MOTOR_out_R_N=	300;//5		//外側モーター用パラメーター 後半	5	5
volatile int			i_MOTOR_in_F_N=		700;//8		//内側モーター用パラメーター　後半	6	6
volatile int			i_MOTOR_in_R_N=		400;//6		//内側モーター用パラメーター　後半	3	3

#define		Cu_FREE_time  		15		//カーブ終了時の後輪フリーの時間(msec）

#define		Cu_BRAKE_time  		10		//カーブ進入時のブレーキ時間 (msec)
#define		Cu_BRAKE_SP 		30		//カーブ進入時にこの速度以上ならブレーキ
#define		Cu_BRAKE			-90		//カーブ進入時のブレーキ（後輪） 
#define		Cu_BRAKE_out		 0		//カーブ進入時のブレーキ(前輪OUT） 
#define		Cu_BRAKE_Fin		-35		//カーブ進入時のブレーキ(前輪IN） 

//-------------------------------------------------------------
//坂
volatile int saka_max = 1; //認識できる坂の数

#define 	Saka_Encoder1  	1200	//上り坂
#define 	Saka_Encoder2  	600	//坂頂上付近
#define 	Saka_Encoder3  	2500//坂上
#define		Saka_Encoder4  	1200//下り坂

volatile int		    i_TOPSPEED_saka	  =		45;		//上り坂
volatile int		    i_TOPSPEED_saka2	=		30;		//坂頂上付近
volatile int		    i_TOPSPEED_saka3	=		50;		//坂上
volatile int		    i_TOPSPEED_saka4	=		50;		//下り坂


//前半(坂上)
volatile int			i_SPEED_DOWN_saka	=		400;//5		//角度によりi_TOPSPEEDを減速 カーブ前半 8 6
volatile int			i_MOTOR_out_R_saka=	 	100;//1		//外側モーター用パラメーター 1	-2
volatile int			i_MOTOR_in_F_saka	=		300;//4		//内側モーター用パラメーター 	2 	1
volatile int			i_MOTOR_in_R_saka	=		100;//-2		//内側モーター用パラメーター -2	-3
//後半(坂上)
volatile int			i_SPEED_DOWN_N_saka=		500;//7		//角度によりi_TOPSPEEDを減速  カーブ後半 11 10
volatile int			i_MOTOR_out_R_N_saka=	  300;//5		//外側モーター用パラメーター 後半	5	5
volatile int			i_MOTOR_in_F_N_saka=		700;//8		//内側モーター用パラメーター　後半	6	6
volatile int			i_MOTOR_in_R_N_saka=		400;//6		//内側モーター用パラメーター　後半	3	3

//-------------------------------------------------------------
//クランク、ハーフ直後の設定値	カーブのパラメータ変更がクランク、ハーフに影響しないようにするため
#define		    TOPSPEED_CH_Len		700		//クランク、ハーフ直後のこの距離未満は以下の設定値で走る　注意：カーブ前半のみ有効

//クランク
volatile int		  i_C_TOPSPEED	=		28;		//クランク(入)
volatile int		  i_C_TOPSPEED2	=		50;		//クランク(出)

//-------------------------------------------------------------
//ハーフ
volatile int		  i_H_TOPSPEED	=		45;		//ハーフ（侵入）
volatile int		  i_H_TOPSPEED2	=		42;		//ハーフ(斜め)

//**********************************************************************
// setup
//**********************************************************************
void setup() {
  // RMC-RA4M1基板上のDIPスイッチ、LED
  pinMode( 25, INPUT );
  pinMode( 26, INPUT );
  pinMode( 13, OUTPUT );
  digitalWrite( 13, 1 );                // D13 LED 消灯

  // モータ
  pinMode( 67, OUTPUT );                // 右前モータ方向
  pinMode( 68, OUTPUT );                // 左前モータ方向
  pinMode( 69, OUTPUT );                // ステアリングモータ方向
  pinMode( 70, OUTPUT );                // 右後モータ方向
  pinMode( 77, OUTPUT );                // 左後モータ方向
  pinMode( 50, OUTPUT );                // 左後モータ フリー・ブレーキ
  pinMode( 51, OUTPUT );                // 右前モータ フリー・ブレーキ
  pinMode( 52, OUTPUT );                // 左前モータ フリー・ブレーキ
  pinMode( 55, OUTPUT );                // 右後モータ フリー・ブレーキ

  setGPTterminal( 6,  3 );                              // P603(GTIOC7A):ステアリングPWM
  setGPTterminal( 6, 10 );                              // P610(GTIOC5B):左前モータPWM
  setGPTterminal( 6,  9 );                              // P609(GTIOC5A):右前モータPWM
  setGPTterminal( 6,  8 );                              // P608(GTIOC4B):右後モータPWM
  setGPTterminal( 1, 15 );                              // P115(GTIOC4A):左後モータPWM
  GTIOC7A = 0;                                          // ステアリングPWMのON幅の設定
  startPWM_GPT7( GTIOCA , DIV1, MOTOR_CYCLE );          // ステアリングPWMの周期設定(GPT7 GTIOCAを使用)
  GTIOC5B = 0;                                          // 左前モータPWMのON幅の設定
  GTIOC5A = 0;                                          // 右前モータPWMのON幅の設定
  startPWM_GPT5( GTIOCA | GTIOCB, DIV1, MOTOR_CYCLE );  // 左右前モータPWMの周期設定(GPT5 GTIOCA,Bを使用)
  GTIOC4B = 0;                                          // 右後モータPWMのON幅の設定
  GTIOC4A = 0;                                          // 右後モータPWMのON幅の設定
  startPWM_GPT4( GTIOCA | GTIOCB, DIV1, MOTOR_CYCLE );  // 左右後モータPWMの周期設定(GPT4 GTIOCA,Bを使用)

  // A/D変換
  ad.useCh( 13 );                       // D39端子 アナログセンサ左
  ad.useCh( 12 );                       // D40端子 アナログセンサ右
  ad.useCh( 23 );                       // D65端子 角度センサ（ポテンショメータ）
  ad.useCh( 18 );                       // D66端子 電圧
  ad.useCh(  8 );                       // D61端子 CN4 5pin　坂センサ（ポテンショメータ）
  ad.useCh(  2 );                       // TSL1401 D17
  ad.start();

  // 圧電スピーカー
  sp.begin( 2 , GTIOCA, 1 , 13 , 4000 ); // GPT2のGTIOCAを使用、端子はP113

  // プッシュスイッチ
  sw4.begin( 45, 1, 1, 1000, 1000 );    // SW4 端子,論理(1=反転),プルアップあり,連続押し2回目の時間,連続の時間
  sw3.begin( 47, 1, 1, 500, 50 );       // SW3
  sw2.begin( 46, 1, 1, 500, 50 );       // SW2
  sw1.begin( 48, 1, 1, 500, 50 );       // SW1
  sw0.begin( 49, 1, 1, 500, 50 );       // SW0

  // センサ基板
  pinMode( 35, INPUT );
  pinMode( 36, INPUT );
  pinMode( 37, INPUT );
  pinMode( 38, INPUT );
  pinMode( 41, INPUT );
  pinMode( 42, INPUT );

  // CN4の入力４端子
  pinMode( 61, INPUT );
  pinMode( 62, INPUT );
  pinMode( 63, INPUT );
  pinMode( 64, INPUT );

  // LED(RGB)
  pinMode(  4, OUTPUT );
  pinMode(  5, OUTPUT );
  pinMode(  6, OUTPUT );
  pinMode(  7, OUTPUT );
  digitalWrite( 6 , 0 );
  led_out( 0 );

  //TSL1401
  pinMode(  8, OUTPUT );
  pinMode(  9, OUTPUT );
  digitalWrite( 8 , 0 );
  digitalWrite( 9 , 0 );

  startGPT0_Encoder( 3, 0 , 1, 8 );     // 2相エンコーダ 0A=P300 0B=P108を使用
  //startGPT0_Encoder( 3, 0 , 0, 0 );   // 1相エンコーダ 0A=P300を使用

  R_ADC0->ADCSR_b.ADCS = 0;
  R_ADC0->ADCER_b.ACE = 0;

  // AGT 1msごとの割り込み処理の設定 PCLKB=24MHz ∴TIMER_SOURCE_DIV_1(1分周)なら、1/(24e6*1) * 24000 = 1ms  設定は１小さい値である23999を設定する
  fsp_timer.begin(TIMER_MODE_PERIODIC, AGT_TIMER, 1, 23999, 1, (timer_source_div_t)TIMER_SOURCE_DIV_1, AGTCallback);
  IRQManager::getInstance().addPeripheral(IRQ_AGT, (void*)fsp_timer.get_cfg());
  fsp_timer.open();

  Serial.begin( 9600 );                 // シリアル通信を使用
#ifdef DEBUG_PRINT
  while( !Serial ) {
    // シリアル接続されるまで待つ
    if( cnt1 >= 3000 ) break;           // タイムアウト
  }
#endif

  Serial.print( "\033[H" );             // デバッグ用画面クリア
  Serial.print( "\033[2J" );

  if( !SD.begin( 2.4e6, CS1 ) ) {       // 通信速度Hz(e6=10の6乗),microSDのCS1(pins_arduino.hで定義)
    f_err_sd = 1;
    Serial.println( "microSDが認識されません!" );       // microSDは認識されず(刺しているのに認識されない場合は2.4e6の値を小さくしてください)
  } else {
    Serial.println( "microSDを認識しました。" );        // microSD認識OK!
    SdFile::dateTimeCallback( &set_microSD_dateTime );  // コールバック関数の定義（日付と時刻を返す関数を登録）
  }

  if( f_err_sd != 0 ) {
    // microSD処理にエラーがあれば音を鳴らし、3秒間LEDの点灯方法を変える
    sp.setSpPattern( 0xcccc );
    cnt1 = 0;
    while( cnt1 < 3000 ) {
      if( cnt1 % 200 < 100 ) {
        led_out( 0x4 );
      } else {
        led_out( 0x0 );
      }
    }
  }

  angle0_ad  = AD_023;                  // 0度の位置記憶

  // マイコンカーの状態初期化
  motor_st( 0 );
  motor_mode_f( BRAKE, BRAKE );
  motor_mode_r( BRAKE, BRAKE );
  motor_f( 0, 0 );
  motor_r( 0, 0 );

  if( df_debug != 0 ) {
    // デバッグモードが１なら音を鳴らし、1.5秒間LEDの点灯方法を変える
    sp.setSpPattern( 0xf0f0 );
    cnt1 = 0;
    while( cnt1 < 1500 ) {
      if( cnt1 % 200 < 100 ) {
        led_out( 0x5 );
      } else {
        led_out( 0x0 );
      }
    }
  }
  
  sp.setSpPattern( 0x8000 );            // 初期化終了
  f_setup_end = 1;                      // setup完了

}

//**********************************************************************
// loop
//**********************************************************************
void loop() {
  int i;

  // microSDへのログ保存処理
  if( f_err_sd == 0 ) {
    switch( log_pattern ) {
      case 0:
        // 待機
        if( log_st == LOG_INIT ) {
          log_pattern = 991;
        }
        break;

      case 991:
        // 連番の読み込み
        i = 0;
        Serial.println( "microSDのrenban.txtを読み込みます。" );
        microSD = SD.open( "renban.txt", FILE_READ );
        if( microSD != 0 ) {
          int length = microSD.available();
          if( length > 8 ){
            length = 8;
          }
          microSD.read( _fn_buf, length );
          sscanf( _fn_buf, "%d", &i );
          if( i < 0 || i >= 99999 ) {
              i = 0;
          }
          microSD.close();
        } else {
          Serial.println( "renban.txtが開けませんので「log00001.csv」でファイルを作成します。" );
        }

        // 連番ファイルを一度、削除
        if( SD.exists( "renban.txt" ) ) {
          SD.remove( "renban.txt" );
        }

        // 連番ファイルを新規に作成
        microSD = SD.open( "renban.txt", FILE_WRITE );
        if( microSD != 0 ) {
          sprintf( _fn_buf, "%d", i + 1 );
          microSD.println( _fn_buf );
          microSD.close();
        }

        // 新しい番号でログファイル作成
        sprintf( _fn_buf, "log%05d.csv", i + 1 );
        microSD = SD.open( _fn_buf, FILE_WRITE );
        if( microSD != 0 ) {
          Serial.print( _fn_buf );
          Serial.println( "でログファイルを作成します。" );
          log_pattern = 992;
        } else {
          f_err_sd = 1;
          Serial.println( "ログファイルが作成できません。" );
          log_pattern = 999;
        }
        break;

      case 992:
        // 最初に書き込む内容
        microSD.println( "ms,pattern,center,wide,angle,m_sv,m_lf,m_rf,m_lr,m_rr,enc,saka_ad" );
        log_st = LOG_INIT_END;
        log_pattern = 993;
        break;

      case 993:
        if( log_st == LOG_WRITE ) {
          log_pattern = 994;
        }
        break;

      case 994:
        if( log_write != log_read ) {    // 保存するログが更新されたらmicroSDにログを書き込み
          char _log_buf[8];
         // sprintf( _log_buf, "%04d", convertBCD_CharToLong( log_buff[log_read].dsensor & 0x0f) );  // センサの値は2進数8桁で表示
          microSD.print( log_buff[log_read].time );
          microSD.print( "," );
          microSD.print( log_buff[log_read].ptn );
          microSD.print( "," );
          microSD.print( log_buff[log_read].tsl1401_Center );
          microSD.print( "," );
          microSD.print( log_buff[log_read].tsl1401_Wide );
          microSD.print( "," );
          microSD.print( log_buff[log_read].angle );
          microSD.print( "," );
          microSD.print( (int)log_buff[log_read].st );
          microSD.print( "," );
          microSD.print( (int)log_buff[log_read].lf );
          microSD.print( "," );
          microSD.print( (int)log_buff[log_read].rf );
          microSD.print( "," );
          microSD.print( (int)log_buff[log_read].lr );
          microSD.print( "," );
          microSD.print( (int)log_buff[log_read].rr );
          microSD.print( "," );
          microSD.print( log_buff[log_read].encoder );
          microSD.print( "," );
          microSD.print( (int)log_buff[log_read].saka_ad );
          microSD.println( "" );

          log_read++;
          if( log_read >= LOG_MAX ) {
            log_read = 0;
          }
        } else if( log_st != LOG_WRITE ) {  // 最新のログが保存されていて、書き込みでないならログ処理終了
          log_pattern = 995;
        }
        break;

      case 995:
        microSD.close();                // 必ずcloseしないと、ファイルが残らない
        log_pattern = 999;
        break;

      case 999:
        // 何もしない
        break;
    }
  }
}

///*****************************************************************
// 1msごとの割り込み処理（サンプルプログラムの実測で30usで終了）
///*****************************************************************
void AGTCallback(timer_callback_args_t __attribute((unused)) * p_args)
{
  static int enc_buff = 0;              // １回前のエンコーダ値
  static int i_Timer10 = 0;
  static int si_Encoder1_buf[10] = {0}; 
  int i,x,r,f;
  unsigned char b;

  ang_old = ang;//前回値の保存
  ang  = getServoAngle();
  saka = getSakaAngle();
  vbat = AD_018 * 5.00 / (16383/3);    // 16383:5.00V = AD_018 : 1/3*VBAT

  cnt_start++;
  cnt1++;
  if( pattern >= 10 && pattern <= 1000 ) {
    cnt_check_sen++;
    cnt_check_enc++;
    ul_cnt_straight_time_1ms++;
	  ul_cnt_curve_time_1ms++;
  }
  //cnt_lcd++;
  //f_lcd = 1;

  sp.sp1msProcess();                    // 圧電スピーカー処理

  if( f_setup_end == 0 ) {              // セットアップ実行中はここで終わり
    return;
  }

  tsl1401(); 
  if(pattern == 0){
    if(tsl1401_Wide == 0){
      led_out(0x00);
    }else if(tsl1401_Wide > 50){
      led_out(0x07);

    }else if(tsl1401_Center < -12){
      led_out(0x01);
    }else if(tsl1401_Center < -4){
      led_out(0x03);

    }else if(tsl1401_Center < 4){
      led_out(0x02);

    }else if(tsl1401_Center < 12){
      led_out(0x06);
    }else{
      led_out(0x04);
    }
  }
  //return;

  servoControl();                       // ステアリングモータ制御（中心線トレース）実測で0.84usで実行
  servoControl2();                      // ステアリングモータ制御（角度）          実測で0.94usで実行

  //Serial.println(saka);

  // 10回中1回実行する処理
  switch( cnt_start % 10 ) {
    case 0:
      if( log_st == LOG_WRITE) {
        log_buff[log_write].time    = cnt_start;
        log_buff[log_write].ptn     = pattern;
        log_buff[log_write].tsl1401_Center = tsl1401_Center;
        log_buff[log_write].tsl1401_Wide = tsl1401_Wide;
        log_buff[log_write].angle   = ang;

        log_buff[log_write].lf      = log_raw.lf;
        log_buff[log_write].rf      = log_raw.rf;
        log_buff[log_write].lr      = log_raw.lr;
        log_buff[log_write].rr      = log_raw.rr;
        log_buff[log_write].st      = log_raw.st;
        log_buff[log_write].encoder = enc;

        log_buff[log_write].saka_ad = saka;
      
        log_write++;
        if( log_write >= LOG_MAX ) {
          log_write = 0;
        }
      }
      break;
  }

  // エンコーダ制御　ここから
  i = INT_GPT0_CNT;
  si_Encoder1_buf[i_Timer10]  = (i - enc_buff)/2;
  enc_total += si_Encoder1_buf[i_Timer10];
  if(enc_total < 0)enc_total = 0;
  if((i - enc_buff)%2 == 0){
    enc_buff = i;
  }else{
    enc_buff = i - 1;
  }
 
  i = 0;
	for(int k = 0; k < 10; k++)i += si_Encoder1_buf[k];
	enc = i;

  i_Timer10++;
  if(i_Timer10 >= 10)i_Timer10 = 0;
  //エンコーダ関連　ここまで

 
 #ifndef DEBUG_PRINT
  if( pattern >= 10 && pattern <= 1000 ) {
    if( df_debug == 0 ) {
      if(cnt_start >=  MAXTIME * 10LL){//走行時間終了
        pattern = 1001;
      }
      
      // 脱輪時の停止処理（カメラ）
      if( tsl1401_Wide  == 0 || check_crossline() == 1 ) {
        if( cnt_check_sen >= 300 ) {
          pattern = 1001;
        }
      } else {
        cnt_check_sen = 0;
      }
  
      if( enc <= 5 || pushsw_get() == 1) {                    // 脱輪時の停止処理（ロータリエンコーダ）|| スイッチ長押し(メモ:電源強制オフではログが保存されない)
        if( cnt_check_enc >= 300 ) {
          pattern = 1001;
        }
      } else {
        cnt_check_enc = 0;
      }

    }
  }
  #endif

  //////////////// 走行処理 ////////////////
  switch( pattern ) {
    case 00:
#if 0 // ステアリングモータの角度制御実験用
      set_angle = +10 * 48;             // +で左 -で右にハンドルを切ります
      motor_st( pwm_kakudo );
      break;
#endif
      // プッシュスイッチ押下待ち
      motor_st( 0 );
      if( pushsw_get() == 1 ) {
        sp.setSpPattern( 0x8000 );
        led_out( 0 );
        led_out_m( 0 );
        pattern = 1;
        cnt1 = 0;
        break;
      }
      
      if( startbar_get() == 1 ) {
        led_out_m( (cnt1%100) < 3 );      // LED点滅 早い
      }else{
        led_out_m( (cnt1%500) < 3 );      // LED点滅　遅い
      }
      break;

    case 01:
      // プッシュスイッチが離されたら
      if( pushsw_get() == 0 && cnt1 >= 50 ) {
        log_st = LOG_INIT;              // ログ処理開始（ファイルオープンなど)
      
        angle0_ad  = AD_023;            // 0度の位置記憶

        cnt1 = 0;
        pattern = 3;
        break;
      }

      break;

    case 03:
      // microSDの初期化待ち
      if( log_st == LOG_INIT_END ) {    // 初期化が終わったら
        pattern = 4;
      }
      if( f_err_sd != 0 ) {
        pattern = 4;
      }
      break;

    case 04:
      // スタートバー開待ち
      motor_st( 0 );        
      led_out( 0x04 >> (cnt1/50) % 4 ); // スタートバー開待ちは、LEDを高速で点滅させる
      if( startbar_get() == 0 ) {       // スタートバーの反応が無くなったら（開いたら）
        led_out( 0x0 );
        cnt_start = 0;
        cnt1 = 0;
        cnt_check_sen = 0;
        cnt_check_enc = 0;
        enc_total = 0;
        enc_kizyun = 0;
        ul_cnt_straight_time_1ms = 0;
        log_st = LOG_WRITE;             // ログ保存開始
        pattern = 10;
      }
      break;

    case 10://スタート直後

      if(enc_total < 200){
        tsl1401_mode = 1;//視野を狭くする

        set_angle = 0;             // +で左 -で右にハンドルを切ります 1度あたり48
        motor_st( pwm_kakudo );

      }else{
        tsl1401_mode = 0;//通常
        motor_st( pwm_trace /2 );
      }
      
      if( enc >= i_TOPSPEED ) {// 直線スピード
          motor_f( 0, 0 );
          motor_r( 0, 0 );
      }else{
          motor_f( 100, 100 );
          motor_r( 100, 100 );
      }

      if(enc_total < 700){
          tsl1401_mode = 0;//通常
          pattern = 11;
          ul_cnt_curve_time_1ms = 0;
      }
      break;

    case 11:
      // 通常トレース
      motor_st( pwm_trace );

      if(ang < -400){//右カーブ

        if((ang - ang_old < 0) && (Cu_flag == 0)){//直線からカーブへ
          if(ul_cnt_straight_time_1ms >= 30 && (enc_total - enc_kizyun > 100)  && (enc > Cu_BRAKE_SP)){//あまり直線を走っていない時はブレーキしないように && クランクなどの直後は無視
					  if(enc_total > 500 && (enc_total - enc_kizyun ) < TOPSPEED_CH_Len ){//クランク、ハーフ直後はブレーキしない
						  ul_cnt_straight_time_1ms = Cu_BRAKE_time + 1;//クランク直後のカーブはまだクランクの抜けなのでブレーキ不要
					  }else{
						  ul_cnt_straight_time_1ms = 0;
					  }
				  }
          Cu_flag = 1;
        }
				
			  if(ul_cnt_straight_time_1ms <= Cu_BRAKE_time && (enc_total - enc_kizyun ) >= 100){//カーブ進入時のブレーキ
				
				  motor_st( pwm_trace *2 );
				
				  motor_f( Cu_BRAKE_out , Cu_BRAKE_Fin );
          motor_r( Cu_BRAKE , Cu_BRAKE );

        }else if(ul_cnt_curve_time_1ms <= Cu_N_time){//カーブ前半
          if(enc >= i_TOPSPEED - (-ang / i_SPEED_DOWN)){
            x=((i_TOPSPEED -(ang / i_SPEED_DOWN))-enc)*2;	
				    r = x;
				    f = x;

				    if(x < -10) x = -10;
				    if(r < -20) r = -20;
				    if(f < -20) f = -20;
				
				    motor_f( x, r );
            motor_r( f, r );	
          }else{
            motor_f( MOTOR_OUT_BASE, MOTOR_OUT_BASE - (-ang / i_MOTOR_in_F) );
            motor_r( MOTOR_OUT_BASE - (-ang / i_MOTOR_out_R), MOTOR_OUT_BASE - (-ang / i_MOTOR_in_R) );
          }

        }else{//カーブ後半
          if(enc >= i_TOPSPEED - (-ang / i_SPEED_DOWN_N)){
            x=((i_TOPSPEED -(ang / i_SPEED_DOWN_N))-enc)*2;	
				    r = x;
				    f = x;

				    if(x < -5) x = -5;
				    if(r < -5) r = -5;
				    if(f < -5) f = -5;
				
				    motor_f( x, r );
            motor_r( f, r );	
          }else{
            motor_f( MOTOR_OUT_BASE, MOTOR_OUT_BASE - (-ang / i_MOTOR_in_F_N) );
            motor_r( MOTOR_OUT_BASE - (-ang / i_MOTOR_out_R_N), MOTOR_OUT_BASE - (-ang / i_MOTOR_in_R_N) );
          }
        }
        

      }else if(400 < ang){//左カーブ

        if((ang - ang_old > 0) && (Cu_flag == 0)){//直線からカーブへ
          if(ul_cnt_straight_time_1ms >= 30 && (enc_total - enc_kizyun > 100)  && (enc > Cu_BRAKE_SP)){//あまり直線を走っていない時はブレーキしないように && クランクなどの直後は無視
					  if(enc_total > 500 && (enc_total - enc_kizyun ) < TOPSPEED_CH_Len ){//クランク、ハーフ直後はブレーキしない
						  ul_cnt_straight_time_1ms = Cu_BRAKE_time + 1;//クランク直後のカーブはまだクランクの抜けなのでブレーキ不要
					  }else{
						  ul_cnt_straight_time_1ms = 0;
					  }
				  }
          Cu_flag = 1;
        }
        
         if(ul_cnt_straight_time_1ms <= Cu_BRAKE_time && (enc_total - enc_kizyun ) >= 100){//カーブ進入時のブレーキ
				
				  motor_st( pwm_trace *2 );
				
				  motor_f( Cu_BRAKE_Fin, Cu_BRAKE_out );
          motor_r( Cu_BRAKE, Cu_BRAKE );

        }else if(ul_cnt_curve_time_1ms <= Cu_N_time){//カーブ前半
          if(enc >= i_TOPSPEED - (ang / i_SPEED_DOWN)){
            x=((i_TOPSPEED -(-ang / i_SPEED_DOWN))-enc)*2;
				    r = x;
				    f = x;	
	
				    if(x < -10) x = -10;
				    if(r < -20) r = -20;
				    if(f < -20) f = -20;
				
				    motor_f( r, x );
            motor_r( r, f );
          }else{
            motor_f( MOTOR_OUT_BASE - (ang / i_MOTOR_in_F), MOTOR_OUT_BASE );
            motor_r( MOTOR_OUT_BASE - (ang / i_MOTOR_in_R), MOTOR_OUT_BASE - (ang / i_MOTOR_out_R) );
          }

        }else{//カーブ後半
          if(enc >= i_TOPSPEED - (ang / i_SPEED_DOWN_N)){
            x=((i_TOPSPEED -(-ang / i_SPEED_DOWN_N))-enc)*2;
				    r = x;
				    f = x;	
	
				    if(x < -5) x = -5;
				    if(r < -5) r = -5;
				    if(f < -5) f = -5;
				
				    motor_f( r, x );
            motor_r( r, f );
          }else{
            motor_f( MOTOR_OUT_BASE - (ang / i_MOTOR_in_F_N), MOTOR_OUT_BASE );
            motor_r( MOTOR_OUT_BASE - (ang / i_MOTOR_in_R_N), MOTOR_OUT_BASE - (ang / i_MOTOR_out_R_N) );
          }
        }
      }else{//直線
        ul_cnt_curve_time_1ms = 0;

        if(Cu_flag == 1){//カーブから直線へ
          if(ul_cnt_straight_time_1ms >= 60){//カーブを一定時間走っていたら＝あまりカーブを走っていない時（後輪滑り後）はフリーにしたくない
					  ul_cnt_straight_time_1ms = 0;
				  }

          Cu_flag = 0;
        }

        if(ul_cnt_straight_time_1ms <= Cu_FREE_time && (enc_total - enc_kizyun > 700)){//カーブ終わりから一定時間内　＆＆　クランク、ハーフからの復帰直後ではない
          if(tsl1401_Center < -10) {//車体左寄り
					  motor_f(95 , 100 );
				
				  }else if(tsl1401_Center > 10) {//車体右寄り
					  motor_f(100 , 95 );
					
				  }else{
					  motor_f(100 , 100 );
				  }
          motor_r( 0, 0 );

        }else if( enc >= i_TOPSPEED ) {// 直線スピード
          motor_f( 0, 0 );
          motor_r( 0, 0 );
        
        }else if(tsl1401_Center < -10) {//車体左寄り
				
				  motor_f(95 , 100 );
				  motor_r(100 , 100 );
			  }else if(tsl1401_Center > 10) {//車体右寄り
				
				  motor_f(100 , 95 );
				  motor_r(100 , 100 );

        } else {
          motor_f( 100, 100 );
          motor_r( 100, 100 );
        }
      }

      if( -1440 < ang && ang < 1440 ) {
        if(enc_kizyun == 0 || (enc_total - enc_kizyun > 300)){
          if( check_crossline() == 1 ) {    // クロスラインチェック
            cnt1 = 0;
            pattern = 21;
            enc_kizyun = enc_total;         // ここを基準とする
            break;
          }
          if( check_rightline() == 1 ) {    // 右ハーフラインチェック
            cnt1 = 0;
            pattern = 51;
            enc_kizyun = enc_total;         // ここを基準とする
            break;
          }
          if( check_leftline() == 1 ) {     // 右ハーフラインチェック
            cnt1 = 0;
            pattern = 61;
            enc_kizyun = enc_total;         // ここを基準とする
            break;
          }
        }
      }


      if(saka_cnt < saka_max){
        if( -1440 < ang && ang < 1440 ) {
          // 上り坂チェック
          if( saka <= 3500) {
            cnt_up++;
          } else {
            cnt_up = 0;
          }

          if( cnt_up >= 10 ) {              // 上り坂チェック
            cnt_up = 0;
            cnt1 = 0;
            saka_cnt++;
            enc_kizyun = enc_total;         // ここを基準とする
            pattern = 101;
            break;
          }
        }else{
          cnt_up = 0;
        }
      }

      break;

    case 101:
      // 上り坂
      motor_st( pwm_trace );

      led_out( 0x6 ); // 黄（赤＋緑）
      

      if( enc >= i_TOPSPEED_saka ) {
        fmotor = (i_TOPSPEED_saka-enc)*1;
			  rmotor = (i_TOPSPEED_saka-enc)*1;
	
			  motor_f( fmotor, fmotor );
        motor_r( rmotor, rmotor );
      } else {
        motor_f( 100, 100 );
        motor_r( 100, 100 );
      }

      if( enc_total - enc_kizyun >= Saka_Encoder1 ) {  // m進んだら
        enc_kizyun = enc_total;         // ここを基準とする
        pattern = 102;
      }
      break;

    case 102:
      // 上り坂 頂上付近なので強めにブレーキ
      motor_st( pwm_trace );

      if( enc >= i_TOPSPEED_saka2 ) {
        fmotor = (i_TOPSPEED_saka2-enc)*5;
			  rmotor = (i_TOPSPEED_saka2-enc)*5;
	
			  motor_f( fmotor, fmotor );
        motor_r( rmotor, rmotor );
      } else {
        motor_f( 50, 50 );
        motor_r( 0, 0 );
      }

      if( enc_total - enc_kizyun >= Saka_Encoder2) {  // m進んだら
        enc_kizyun = enc_total;         // ここを基準とする
        pattern = 103;
      }
      break;
  
    case 103:
      // 坂上
      motor_st( pwm_trace );

      if(ang < -400){//右カーブ

        if((ang - ang_old < 0) && (Cu_flag == 0)){//直線からカーブへ
          if(ul_cnt_straight_time_1ms >= 30 && (enc_total - enc_kizyun > 100)  && (enc > Cu_BRAKE_SP)){//あまり直線を走っていない時はブレーキしないように && クランクなどの直後は無視
					  if(enc_total > 500 && (enc_total - enc_kizyun ) < TOPSPEED_CH_Len ){//クランク、ハーフ直後はブレーキしない
						  ul_cnt_straight_time_1ms = Cu_BRAKE_time + 1;//クランク直後のカーブはまだクランクの抜けなのでブレーキ不要
					  }else{
						  ul_cnt_straight_time_1ms = 0;
					  }
				  }
          Cu_flag = 1;
        }
				
			  if(ul_cnt_straight_time_1ms <= Cu_BRAKE_time && (enc_total - enc_kizyun ) >= 100){//カーブ進入時のブレーキ
				
				  motor_st( pwm_trace *2 );
				
				  motor_f( Cu_BRAKE_out , Cu_BRAKE_Fin );
          motor_r( Cu_BRAKE , Cu_BRAKE );

        }else if(ul_cnt_curve_time_1ms <= Cu_N_time){//カーブ前半
          if(enc >= i_TOPSPEED_saka3 - (-ang / i_SPEED_DOWN_saka)){
            x=((i_TOPSPEED_saka3 -(ang / i_SPEED_DOWN_saka))-enc)*2;	
				    r = x;
				    f = x;

				    if(x < -10) x = -10;
				    if(r < -20) r = -20;
				    if(f < -20) f = -20;
				
				    motor_f( x, r );
            motor_r( f, r );	
          }else{
            motor_f( MOTOR_OUT_BASE, MOTOR_OUT_BASE - (-ang / i_MOTOR_in_F_saka) );
            motor_r( MOTOR_OUT_BASE - (-ang / i_MOTOR_out_R_saka), MOTOR_OUT_BASE - (-ang / i_MOTOR_in_R_saka) );
          }

        }else{//カーブ後半
          if(enc >= i_TOPSPEED_saka3 - (-ang / i_SPEED_DOWN_N_saka)){
            x=((i_TOPSPEED_saka3 -(ang / i_SPEED_DOWN_N_saka))-enc)*2;	
				    r = x;
				    f = x;

				    if(x < -5) x = -5;
				    if(r < -5) r = -5;
				    if(f < -5) f = -5;
				
				    motor_f( x, r );
            motor_r( f, r );	
          }else{
            motor_f( MOTOR_OUT_BASE, MOTOR_OUT_BASE - (-ang / i_MOTOR_in_F_N_saka) );
            motor_r( MOTOR_OUT_BASE - (-ang / i_MOTOR_out_R_N_saka), MOTOR_OUT_BASE - (-ang / i_MOTOR_in_R_N_saka) );
          }
        }
        

      }else if(400 < ang){//左カーブ

        if((ang - ang_old > 0) && (Cu_flag == 0)){//直線からカーブへ
          if(ul_cnt_straight_time_1ms >= 30 && (enc_total - enc_kizyun > 100)  && (enc > Cu_BRAKE_SP)){//あまり直線を走っていない時はブレーキしないように && クランクなどの直後は無視
					  if(enc_total > 500 && (enc_total - enc_kizyun ) < TOPSPEED_CH_Len ){//クランク、ハーフ直後はブレーキしない
						  ul_cnt_straight_time_1ms = Cu_BRAKE_time + 1;//クランク直後のカーブはまだクランクの抜けなのでブレーキ不要
					  }else{
						  ul_cnt_straight_time_1ms = 0;
					  }
				  }
          Cu_flag = 1;
        }
        
         if(ul_cnt_straight_time_1ms <= Cu_BRAKE_time && (enc_total - enc_kizyun ) >= 100){//カーブ進入時のブレーキ
				
				  motor_st( pwm_trace *2 );
				
				  motor_f( Cu_BRAKE_Fin, Cu_BRAKE_out );
          motor_r( Cu_BRAKE, Cu_BRAKE );

        }else if(ul_cnt_curve_time_1ms <= Cu_N_time){//カーブ前半
          if(enc >= i_TOPSPEED_saka3 - (ang / i_SPEED_DOWN_saka)){
            x=((i_TOPSPEED_saka3 -(-ang / i_SPEED_DOWN_saka))-enc)*2;
				    r = x;
				    f = x;	
	
				    if(x < -10) x = -10;
				    if(r < -20) r = -20;
				    if(f < -20) f = -20;
				
				    motor_f( r, x );
            motor_r( r, f );
          }else{
            motor_f( MOTOR_OUT_BASE - (ang / i_MOTOR_in_F_saka), MOTOR_OUT_BASE );
            motor_r( MOTOR_OUT_BASE - (ang / i_MOTOR_in_R_saka), MOTOR_OUT_BASE - (ang / i_MOTOR_out_R_saka) );
          }

        }else{//カーブ後半
          if(enc >= i_TOPSPEED_saka3 - (ang / i_SPEED_DOWN_N_saka)){
            x=((i_TOPSPEED_saka3 -(-ang / i_SPEED_DOWN_N_saka))-enc)*2;
				    r = x;
				    f = x;	
	
				    if(x < -5) x = -5;
				    if(r < -5) r = -5;
				    if(f < -5) f = -5;
				
				    motor_f( r, x );
            motor_r( r, f );
          }else{
            motor_f( MOTOR_OUT_BASE - (ang / i_MOTOR_in_F_N_saka), MOTOR_OUT_BASE );
            motor_r( MOTOR_OUT_BASE - (ang / i_MOTOR_in_R_N_saka), MOTOR_OUT_BASE - (ang / i_MOTOR_out_R_N_saka) );
          }
        }
      }else{//直線
        ul_cnt_curve_time_1ms = 0;

        if(Cu_flag == 1){//カーブから直線へ
          if(ul_cnt_straight_time_1ms >= 60){//カーブを一定時間走っていたら＝あまりカーブを走っていない時（後輪滑り後）はフリーにしたくない
					  ul_cnt_straight_time_1ms = 0;
				  }

          Cu_flag = 0;
        }

       if( enc >= i_TOPSPEED_saka3 ) {// 直線スピード
          motor_f( 0, 0 );
          motor_r( 0, 0 );
        
        } else {
          motor_f( 100, 100 );
          motor_r( 100, 100 );
        }
      }

      if( enc_total - enc_kizyun >= Saka_Encoder3 ) {  // m進んだら
        enc_kizyun = enc_total;         // ここを基準とする
        pattern = 104;
      }
      break;

    case 104:
      // 下り坂
      motor_st( pwm_trace );

      if( enc >= i_TOPSPEED_saka4 ) {
        motor_f( 0, 0 );
        motor_r( 0, 0 );
      } else {
        motor_f( 100, 100 );
        motor_r( 100, 100 );
      }

      if( enc_total - enc_kizyun >= Saka_Encoder4 ) {  // m進んだら
        enc_kizyun = enc_total;         // ここを基準とする
        pattern = 11;
        led_out( 0x0 );
      }
      break;

    case 21:
      led_out( 0x1 );

      // クロスライン通過処理
      if(tsl1401_Wide > 20 || tsl1401_Center < -20 || tsl1401_Center > 20 ){//クロスラインが見えているとき
			  set_angle = 0;             // +で左 -で右にハンドルを切ります 1度あたり48
        motor_st( pwm_kakudo );
		  }else{
			  motor_st( pwm_trace );
		  } 

      if( (enc >= i_C_TOPSPEED) ) {          // エンコーダによりスピード制御 
          
			  fmotor = (i_C_TOPSPEED-enc)*10;
			  rmotor = (i_C_TOPSPEED-enc)*10;
			 
        motor_mode_f( BRAKE, BRAKE );
			  motor_mode_r( BRAKE, BRAKE );
			
			  motor_f( fmotor, fmotor );
        motor_r( rmotor, rmotor );
		 
      }else{
			  motor_mode_f( FREE, FREE );
			  motor_mode_r( FREE, FREE );
				
        motor_f( 100, 100 );
        motor_r( 100, 100 );
      }	

      if( cnt1 >= 50 ) {
        cnt1 = 0;
        motor_mode_f( FREE, FREE );
			  motor_mode_r( FREE, FREE );
        pattern = 22;
      }
      break;

    case 22:
      // クロスライン後のトレース、直角検出処理
      motor_st( pwm_trace );

      if( (enc >= i_C_TOPSPEED) ) {          // エンコーダによりスピード制御 
          
			  fmotor = (i_C_TOPSPEED-enc)*10;
			  rmotor = (i_C_TOPSPEED-enc)*10;
			 
        motor_mode_f( BRAKE, BRAKE );
			  motor_mode_r( BRAKE, BRAKE );
			
			  motor_f( fmotor, fmotor );
        motor_r( rmotor, rmotor );
		 
      }else{
			  motor_mode_f( FREE, FREE );
			  motor_mode_r( FREE, FREE );
				
        motor_f( 100, 100 );
        motor_r( 100, 100 );
      }	

      if((enc_total - enc_kizyun > 250)){
        if( check_rightline_forC() == 1 ) { // 右クランク
          led_out( 0x2 );
          cnt1 = 0;
          enc_kizyun = enc_total;         // ここを基準とする
          pattern = 31;
          break;
        }
        if( check_leftline_forC() == 1 ) {  // 左クランク
          led_out( 0x4 );
          cnt1 = 0;
          enc_kizyun = enc_total;         // ここを基準とする
          pattern = 41;
          break;
        }
      }

      if((enc_total - enc_kizyun > 2000)){//誤動作チェック
        motor_mode_f( FREE, FREE );
			  motor_mode_r( FREE, FREE );

        led_out( 0x0 );
        cnt1 = 0;
        pattern = 11;
      }
      break;

    case 31:
      // 右クランク処理
      tsl1401_mode = 3;//左無視

      set_angle = 45 * 48;             // +で左 -で右にハンドルを切ります 1度あたり48
      motor_st( pwm_kakudo );

      motor_f( 60,  0 );
      motor_r( 49, 22 );

      if((enc_total - enc_kizyun > 150)){// 曲げ終わりチェック
        if(tsl1401_Wide != 0){
          if(18 < tsl1401_Center && tsl1401_Center < 35 && (tsl1401_Wide != 0 && tsl1401_Wide < 12)  ){
            cnt1 = 0;
            enc_kizyun = enc_total;         // ここを基準とする
            pattern = 32;
          }
        }
      }
      break;

    case 32:
      // 少し時間が経つまで待つ
      if((enc_total - enc_kizyun > 50)){
        tsl1401_mode = 0;//通常
      }

      set_angle = 30 * 48;             // +で左 -で右にハンドルを切ります 1度あたり48
      motor_st( pwm_kakudo );

      motor_r( 80, 80 );
      motor_f( 80, 80 );

      if((enc_total - enc_kizyun > 50)){
        if(tsl1401_Wide != 0){
          if( -15 < tsl1401_Center && tsl1401_Center < 15  ){
            
            cnt1 = 0;
            enc_kizyun = enc_total;         // ここを基準とする
            pattern = 33;
            tsl1401_mode = 0;//通常
          }
        }
      }
      
      break;

    case 33:
      //少し待つ
      motor_st( pwm_trace );

      if( enc >= i_TOPSPEED ) { // エンコーダによりスピード制御   
			  fmotor=(i_TOPSPEED-enc)*10;
			  motor_f( fmotor, fmotor);
        motor_r( fmotor, fmotor );

      }else{
			  motor_f( 100, 100 );
        motor_r( 70, 70 );
		  }

      if((enc_total - enc_kizyun > 100)){
        if(tsl1401_Wide != 0){
          if( -90 < ang && ang < 90  ){
            
            cnt1 = 0;
            enc_kizyun = enc_total;         // ここを基準とする
            led_out( 0x0 );
            pattern = 11;
          }
        }
      }

      break;

    case 41:
      // 左クランク処理
      tsl1401_mode = 2;//右無視

      set_angle = -45 * 48;             // +で左 -で右にハンドルを切ります 1度あたり48
      motor_st( pwm_kakudo );

      motor_f( 0,  60 );
      motor_r( 22, 49 );

      if((enc_total - enc_kizyun > 150)){// 曲げ終わりチェック
        if(tsl1401_Wide != 0){
          if(-35 < tsl1401_Center && tsl1401_Center < -18 && (tsl1401_Wide != 0 && tsl1401_Wide < 12)  ){
            cnt1 = 0;
            enc_kizyun = enc_total;         // ここを基準とする
            pattern = 42;
          }
        }
      }
      break;

    case 42:
      // 少し時間が経つまで待つ
      if((enc_total - enc_kizyun > 50)){
        tsl1401_mode = 0;//通常
      }

      set_angle = -30 * 48;             // +で左 -で右にハンドルを切ります 1度あたり48
      motor_st( pwm_kakudo );

      motor_r( 80, 80 );
      motor_f( 80, 80 );

      if((enc_total - enc_kizyun > 50)){
        if(tsl1401_Wide != 0){
          if( -15 < tsl1401_Center && tsl1401_Center < 15  ){
            
            cnt1 = 0;
            enc_kizyun = enc_total;         // ここを基準とする
            pattern = 43;
            tsl1401_mode = 0;//通常
          }
        }
      }
      break;

    case 51:
      // 左レーンチェンジ　左ハーフラインを通り過ぎるまで待つ
      set_angle = 0 * 48;             // +で左 -で右にハンドルを切ります 1度あたり48
      motor_st( pwm_kakudo );

      if(enc > i_H_TOPSPEED ){
        x=(i_H_TOPSPEED-enc)*15;
			  r=(i_H_TOPSPEED-enc)*5;
			
			  motor_f( x, x );
        motor_r( r, r );
      }else{
        motor_f( 100, 100 );
        motor_r( 100, 100 );
      }

      if((enc_total - enc_kizyun > 100)){
        pattern = 52;
        cnt1 = 0;
        enc_kizyun = enc_total;         // ここを基準とする
        break;
      }

      if( check_crossline() == 1 || check_rightline() == 1) {    // クロスラインチェック
        cnt1 = 0;
        pattern = 21;
      }
      break;

    case 52:
      // 左レーンチェンジ　中心線がなくなるまで進む
      motor_st( pwm_trace );

      if(enc > i_H_TOPSPEED ){
        x=(i_H_TOPSPEED-enc)*15;
			  r=(i_H_TOPSPEED-enc)*5;
			
			  motor_f( x, x );
        motor_r( r, r );
      }else{
        motor_f( 100, 100 );
        motor_r( 100, 100 );
      }
      
      if((tsl1401_Wide == 0 || check_crossline() == 1 ) && (enc_total - enc_kizyun > 250) ){
        pattern = 53;
        cnt1 = 0;
        enc_kizyun = enc_total;         // ここを基準とする
      }

      if(enc_total - enc_kizyun > 1500){//誤動作チェック
        pattern = 11;
        cnt1 = 0;
      }
      break;

    case 53:
      // 左レーンチェンジ　新しい中心線が見つかるまで曲げる

      tsl1401_mode = 2;//右無視

      set_angle = -40 * 48;             // +で左 -で右にハンドルを切ります 1度あたり48
      motor_st( pwm_kakudo );

      if(enc > i_H_TOPSPEED2 ){
        x=(i_H_TOPSPEED2-enc)*10;
			  r=(i_H_TOPSPEED-enc)*10;
			
			  motor_f( x, x );
        motor_r( r, r );
      }else{
        motor_f( 0, 100 );
        motor_r( 10, 20 );
      }

      if(enc_total - enc_kizyun > 200){
        if((-50 < tsl1401_Center)&&(tsl1401_Center < -10) && tsl1401_Wide != 0 ) { 
          pattern = 54;
          cnt1 = 0;
          tsl1401_mode = 0;//通常
          enc_kizyun = enc_total;         // ここを基準とする
        }
      }

      break;

    case 54:
      // 左レーンチェンジ　
      if((tsl1401_Center == 0)&&(tsl1401_Wide == 0)){ //インに落ちそう
				set_angle = -50 * 48;             // +で左 -で右にハンドルを切ります 1度あたり48
			
      }else{
				set_angle = -30 * 48;             // +で左 -で右にハンドルを切ります 1度あたり48
      }
      motor_st( pwm_kakudo );

      if(enc > i_H_TOPSPEED2 ){
        x=(i_H_TOPSPEED2-enc)*10;
			  r=(i_H_TOPSPEED-enc)*10;
			
			  motor_f( x, x );
        motor_r( r, r );
      }else{
        motor_f( 100, 100 );
        motor_r( 60, 60 );
      }

      if(10 < tsl1401_Center && tsl1401_Wide != 0  ) {//曲げ終わりチェック
        pattern = 55;
        cnt1 = 0;
        enc_kizyun = enc_total;         // ここを基準とする
      }
      break;

    case 55:
      // 左レーンチェンジ　安定するまで新しい中心線をトレース
      if(tsl1401_Wide != 0 && -10 < tsl1401_Center && tsl1401_Center < 10 ){
        motor_st( pwm_trace );
      }else{
        set_angle = 20 * 48;             // +で左 -で右にハンドルを切ります 1度あたり48
        motor_st( pwm_kakudo );
      }
      
      if(enc > i_H_TOPSPEED2 ){
        x=(i_H_TOPSPEED2-enc)*10;
			  r=(i_H_TOPSPEED-enc)*10;
			
			  motor_f( x, x );
        motor_r( r, r );
      }else{
        motor_f( 100, 30 );
        motor_r( 90, 0 );
      }

      if(enc_total - enc_kizyun > 50){
        if((-20 < tsl1401_Center)&&(tsl1401_Center < -20) && tsl1401_Wide != 0 ) { 
          pattern = 11;
          cnt1 = 0;
          enc_kizyun = enc_total;         // ここを基準とする
        }
      }

      break;

     case 61:
      // 右レーンチェンジ　右ハーフラインを通り過ぎるまで待つ
      set_angle = 0 * 48;             // +で左 -で右にハンドルを切ります 1度あたり48
      motor_st( pwm_kakudo );

      if(enc > i_H_TOPSPEED ){
        x=(i_H_TOPSPEED-enc)*15;
			  r=(i_H_TOPSPEED-enc)*5;
			
			  motor_f( x, x );
        motor_r( r, r );
      }else{
        motor_f( 100, 100 );
        motor_r( 100, 100 );
      }
     
      if( check_crossline() == 1 || check_leftline() == 1) {    // クロスラインチェック
        cnt1 = 0;
        pattern = 21;
        break;
      }
       
      if((enc_total - enc_kizyun > 100)){
        pattern = 62;
        cnt1 = 0;
        enc_kizyun = enc_total;         // ここを基準とする
      }
      break;

    case 62:
      // 右レーンチェンジ　中心線がなくなるまで進む
      motor_st( pwm_trace );

      if(enc > i_H_TOPSPEED ){
        x=(i_H_TOPSPEED-enc)*15;
			  r=(i_H_TOPSPEED-enc)*5;
			
			  motor_f( x, x );
        motor_r( r, r );
      }else{
        motor_f( 100, 100 );
        motor_r( 100, 100 );
      }
      
      if((tsl1401_Wide == 0 || check_crossline() == 1 ) && (enc_total - enc_kizyun > 250) ){
        pattern = 63;
        cnt1 = 0;
        enc_kizyun = enc_total;         // ここを基準とする
      }

      if(enc_total - enc_kizyun > 1500){//誤動作チェック
        pattern = 11;
        cnt1 = 0;
      }
      break;

    case 63:
      // 右レーンチェンジ　新しい中心線が見つかるまで曲げる

      tsl1401_mode = 2;//右無視

      set_angle = 40 * 48;             // +で左 -で右にハンドルを切ります 1度あたり48
      motor_st( pwm_kakudo );

      if(enc > i_H_TOPSPEED2 ){
        x=(i_H_TOPSPEED2-enc)*10;
			  r=(i_H_TOPSPEED-enc)*10;
			
			  motor_f( x, x );
        motor_r( r, r );
      }else{
        motor_f( 100, 0 );
        motor_r( 20, 10 );
      }

      if(enc_total - enc_kizyun > 200){
        if((10 < tsl1401_Center)&&(tsl1401_Center < 50) && tsl1401_Wide != 0 ) { 
          pattern = 64;
          cnt1 = 0;
          tsl1401_mode = 0;//通常
          enc_kizyun = enc_total;         // ここを基準とする
        }
      }

      break;

    case 64:
      // 右レーンチェンジ　
      if((tsl1401_Center == 0)&&(tsl1401_Wide == 0)){ //インに落ちそう
				set_angle = 50 * 48;             // +で左 -で右にハンドルを切ります 1度あたり48
			
      }else{
				set_angle = 30 * 48;             // +で左 -で右にハンドルを切ります 1度あたり48
      }
      motor_st( pwm_kakudo );

      if(enc > i_H_TOPSPEED2 ){
        x=(i_H_TOPSPEED2-enc)*10;
			  r=(i_H_TOPSPEED-enc)*10;
			
			  motor_f( x, x );
        motor_r( r, r );
      }else{
        motor_f( 100, 100 );
        motor_r( 60, 60 );
      }

      if(tsl1401_Center < -10 && tsl1401_Wide != 0  ) {//曲げ終わりチェック
        pattern = 65;
        cnt1 = 0;
        enc_kizyun = enc_total;         // ここを基準とする
      }
      break;

    case 65:
      // 左レーンチェンジ　安定するまで新しい中心線をトレース
      if(tsl1401_Wide != 0 && -10 < tsl1401_Center && tsl1401_Center < 10 ){
        motor_st( pwm_trace );
      }else{
        set_angle = -20 * 48;             // +で左 -で右にハンドルを切ります 1度あたり48
        motor_st( pwm_kakudo );
      }
      
      if(enc > i_H_TOPSPEED2 ){
        x=(i_H_TOPSPEED2-enc)*10;
			  r=(i_H_TOPSPEED-enc)*10;
			
			  motor_f( x, x );
        motor_r( r, r );
      }else{
        motor_f( 30, 100 );
        motor_r( 0, 90 );
      }

      if(enc_total - enc_kizyun > 50){
        if((-20 < tsl1401_Center)&&(tsl1401_Center < -20) && tsl1401_Wide != 0 ) { 
          pattern = 11;
          cnt1 = 0;
          enc_kizyun = enc_total;         // ここを基準とする
        }
      }

      break;



 

    case 1001:
      // 停止処理
      motor_st( pwm_trace );
      motor_f( 0, 0 );
      motor_r( 0, 0 );
      led_out( 0x0 );
      pattern = 1002;
      cnt1 = 0;
      break;

    case 1002:
      motor_st( pwm_trace );
      if( enc <= 5 ) {
        motor_st( 0 );
        pattern = 1003;
        sp.setSpPattern( 0xf0f0 );
        log_st = LOG_END;
        cnt1 = 0;
        break;
      }
      break;

    case 1003:

      break;

    default:
      break;
  }

}

///*****************************************************************
// アナログセンサ基板TypeSのデジタルセンサ値読み込み
// 引数　 なし
// 戻り値 左端、左中、右中、右端のデジタルセンサ 0:黒 1:白
///*****************************************************************
unsigned char sensor_inp( void )
{
  unsigned char c;

  c  = (!R_PORT0->PIDR_b.PIDR12) << 3;
  c |= (!R_PORT0->PIDR_b.PIDR11) << 2;
  c |= (!R_PORT0->PIDR_b.PIDR10) << 1;
  c |= (!R_PORT0->PIDR_b.PIDR8 ) << 0;

  return c;
}

///*****************************************************************
// アナログセンサ基板TypeSの中心デジタルセンサ読み込み
// 引数　 なし
// 戻り値 中心デジタルセンサ 0:黒 1:白
///*****************************************************************
unsigned char center_inp( void )
{
  unsigned char sensor = (!R_PORT0->PIDR_b.PIDR5 );

  return sensor;
}

///*****************************************************************
// アナログセンサ基板TypeSのスタートバー検出センサ読み込み
// 引数　 なし
// 戻り値 0:スタートバーなし 1:スタートバーあり
///*****************************************************************
unsigned char startbar_get( void )
{
  //unsigned char sensor = (!R_PORT0->PIDR_b.PIDR4 );
  //return sensor;

  if(tsl1401_Wide > 40)return 1;
  return 0;
  
}

///*****************************************************************
// マイコンボード上のディップスイッチ(2bit)値読み込み
// 引数　 なし
// 戻り値 スイッチ値 0～3
///*****************************************************************
unsigned char dipsw_get( void )
{
  unsigned char sw = ( !(R_PORT3->PIDR_b.PIDR6) << 1 ) + ( !(R_PORT3->PIDR_b.PIDR7) );

  return sw;
}

///*****************************************************************
// プッシュスイッチ値読み込み
// 引数　 なし
// 戻り値 スイッチ値 0:OFF 1:ON
///*****************************************************************
unsigned char pushsw_get( void )
{
  unsigned char sw = !(R_PORT7->PIDR_b.PIDR8);

  return sw;
}

///*****************************************************************
// CN4の状態読み込み
// 引数　 なし
// 戻り値 0～15
///*****************************************************************
unsigned char get_cn4( void )
{
  unsigned char c;

  c  = (R_PORT0->PIDR_b.PIDR13) << 3;
  c |= (R_PORT0->PIDR_b.PIDR15) << 2;
  c |= (R_PORT5->PIDR_b.PIDR5)  << 1;
  c |= (R_PORT5->PIDR_b.PIDR4)  << 0;

  return c;
}

///*****************************************************************
// フルカラーLEDの制御(LED基板D2～D7にを取り付けてください)
// 引数　 3個のLED制御 0:OFF 1:ON  bit2:赤 bit1:緑 bit0:青
// 戻り値 なし
///*****************************************************************
void led_out( unsigned char led )
{
  R_PORT1->PODR_b.PODR7 = ( (led & 0x4) != 0 );  // 式が真なら1、偽なら0の値を持つ
  R_PORT1->PODR_b.PODR3 = ( (led & 0x2) != 0 );
  R_PORT1->PODR_b.PODR2 = ( (led & 0x1) != 0 );
}

///*****************************************************************
// マイコン搭載のD13 LEDの制御
// 引数　 0:OFF 1:ON
// 戻り値 なし
///*****************************************************************
void led_out_m( unsigned char led )
{
  R_PORT1->PODR_b.PODR11 = ( led == 0 );  // 式が真なら1、偽なら0の値を持つ
}

///*****************************************************************
// 後輪の速度制御 ディップスイッチには関係しないmotor関数
// 引数　 左モータ:-100～100 , 右モータ:-100～100
//        0で停止、100で正転100%、-100で逆転100%
// 戻り値 なし
///*****************************************************************
void motor_r( int accele_l, int accele_r )
{
  log_raw.lr = accele_l;
  log_raw.rr = accele_r;

  if( df_debug != 0 ) {   // デバッグモードならモータ0%
    accele_l = 0;
    accele_r = 0;
  }

  // 左モータ制御
  if( accele_l > 0 ) {
    R_PORT1->PODR_b.PODR14 = 0;
    accele_l = (long)(MOTOR_CYCLE + 1) * accele_l / 100;
    GTIOC4A = accele_l;
  } else if( accele_l < 0 ) {
    R_PORT1->PODR_b.PODR14 = 1;
    accele_l = (long)(MOTOR_CYCLE + 1) * (-accele_l) / 100;
    GTIOC4A = accele_l;
  } else {  // 0%
    GTIOC4A = 0;
  }

  // 右モータ制御
  if( accele_r > 0 ) {
    R_PORT6->PODR_b.PODR1 = 0;
    accele_r = (long)(MOTOR_CYCLE + 1) * accele_r / 100;
    GTIOC4B = accele_r;
  } else if( accele_r < 0 ) {
    R_PORT6->PODR_b.PODR1 = 1;
    accele_r = (long)(MOTOR_CYCLE + 1) * (-accele_r) / 100;
    GTIOC4B = accele_r;
  } else {  // 0%
    GTIOC4B = 0;
  }
}


///*****************************************************************
// ディップスイッチには関係しないmotor関数
// 引数　 左モータ:-100～100 , 右モータ:-100～100
//        0で停止、100で正転100%、-100で逆転100%
// 戻り値 なし
///*****************************************************************
void motor_f( int accele_l, int accele_r )
{
  log_raw.lf = accele_l;
  log_raw.rf = accele_r;

  if( df_debug != 0 ) {   // デバッグモードならモータ0%
    accele_l = 0;
    accele_r = 0;
  }

  // 左モータ制御
  if( accele_l > 0 ) {
    R_PORT5->PODR_b.PODR0 = 0;
    accele_l = (long)(MOTOR_CYCLE + 1) * accele_l / 100;
    GTIOC5B = accele_l;
  } else if( accele_l < 0 ) {
    R_PORT5->PODR_b.PODR0 = 1;
    accele_l = (long)(MOTOR_CYCLE + 1) * (-accele_l) / 100;
    GTIOC5B = accele_l;
  } else {
    GTIOC5B = 0;
  }

  // 右モータ制御
  if( accele_r > 0 ) {
    R_PORT5->PODR_b.PODR1 = 0;
    accele_r = (long)(MOTOR_CYCLE + 1) * accele_r / 100;
    GTIOC5A = accele_r;
  } else if( accele_r < 0 ) {
    R_PORT5->PODR_b.PODR1 = 1;
    accele_r = (long)(MOTOR_CYCLE + 1) * (-accele_r) / 100;
    GTIOC5A = accele_r;
  } else {
    GTIOC5A = 0;
  }
}

///*****************************************************************
// 後モータ停止動作（ブレーキ、フリー）
// 引数　 左モータ:FREE or BRAKE , 右モータ:FREE or BRAKE
// 戻り値 なし
///*****************************************************************
void motor_mode_r( int mode_l, int mode_r )
{
  if( mode_l != 0 ) {
    R_PORT4->PODR_b.PODR11 = 1;
  } else {
    R_PORT4->PODR_b.PODR11 = 0;
  }
  if( mode_r != 0 ) {
    R_PORT2->PODR_b.PODR6 = 1;
  } else {
    R_PORT2->PODR_b.PODR6 = 0;
  }
}

///*****************************************************************
// 前モータ停止動作（ブレーキ、フリー）
// 引数　 左モータ:FREE or BRAKE , 右モータ:FREE or BRAKE
// 戻り値 なし
///*****************************************************************
void motor_mode_f( int mode_l, int mode_r )
{
  if( mode_l != 0 ) {
    R_PORT4->PODR_b.PODR9 = 1;
  } else {
    R_PORT4->PODR_b.PODR9 = 0;
  }
  if( mode_r != 0 ) {
    R_PORT4->PODR_b.PODR10 = 1;
  } else {
    R_PORT4->PODR_b.PODR14 = 0;
  }
}

///*****************************************************************
// ステアリングモータ制御(旧名称servoPwmOut)
// 引数　 ステアリングモータPWM：-100～100
//        0で停止、100で正転100%、-100で逆転100%
// 戻り値 なし
///*****************************************************************
void motor_st( int pwm )
{
  int i = getServoAngle();

  // ボリューム値により左リミット制御
  if( i >= 6000 && pattern >= 11 ) {
    if( pwm < -10 ) pwm = 0;
  }
  // ボリューム値により右リミット制御
  if( i <= -6000 && pattern >= 11 ) {
    if( pwm >  10 ) pwm = 0;
  }

  if( pwm >  100 ) pwm =  100;            // 上限チェック
  if( pwm < -100 ) pwm = -100;            // 下限チェック

  log_raw.st = pwm;                     // ログ用データの保存

  pwm = -pwm;//ステアリングの配線向きが違ったのでソフトで反転

  if( pwm > 0 ) {
    R_PORT6->PODR_b.PODR0 = 0;
    pwm = (long)(MOTOR_CYCLE + 1) * pwm / 100;
    GTIOC7A = pwm;
  } else if( pwm < 0 ) {
    R_PORT6->PODR_b.PODR0 = 1;
    pwm = (long)(MOTOR_CYCLE + 1) * (-pwm) / 100;
    GTIOC7A = pwm;
  } else {
    GTIOC7A = 0;
  }
}

//**********************************************************************
// クロスライン検出処理
// 引数　 なし
// 戻り値 0:クロスラインなし 1:あり
//**********************************************************************
int check_crossline( void )
{
    int ret = 0;

    if( (tsl1401_Wide > 60) || ((tsl1401_Wide >= 40) && (-10 < tsl1401_Center ) && (tsl1401_Center < 10))  ){
		  ret = 1;			/* クロスライン発見 */
	  }
    return ret;
}

//**********************************************************************
// 右ハーフライン検出処理
// 引数　 なし
// 戻り値 0:右ハーフラインなし 1:あり
//**********************************************************************
int check_rightline( void )
{
    int ret = 0;

    if(tsl1401_Wide > 44 && tsl1401_Wide < 50){
		  if(tsl1401_Center > 12){//センター右寄り
			  ret = 1;
		  }
	  }else if(tsl1401_Wide > 34){
		  if(tsl1401_Center > 7){//センター右寄り
			  ret = 1;
		  }
	  }else if(tsl1401_Wide > 28){
		  if(tsl1401_Center > 5){//センター右寄り
			  ret = 1;
		  }
	  }
    return ret;
}

//**********************************************************************
// 左ハーフライン検出処理
// 引数　 なし
// 戻り値 0:左ハーフラインなし 1:あり
//**********************************************************************
int check_leftline( void )
{
    int ret = 0;

    if(tsl1401_Wide > 44 && tsl1401_Wide < 50){
		  if(tsl1401_Center < -12){//センター左寄り
			  ret = 1;
		  }
	  }else if(tsl1401_Wide > 34){
		  if(tsl1401_Center < -7){//センター左寄り
			  ret = 1;
		  }
	  }else if(tsl1401_Wide > 28){
		  if(tsl1401_Center < -5){//センター左寄り
			  ret = 1;
		  }
	  }
    return ret;
}

//**********************************************************************
// 右ハーフライン検出処理  クランク用
// 引数　 なし
// 戻り値 0:右ハーフラインなし 1:あり
//**********************************************************************
int check_rightline_forC( void )
{
    int ret = 0;

    if(tsl1401_Wide > 28){
		  if(tsl1401_Center > 1){//センター右寄り
			  ret = 1;
		  }
	  }
    return ret;
}

//**********************************************************************
// 左ハーフライン検出処理　クランク用
// 引数　 なし
// 戻り値 0:左ハーフラインなし 1:あり
//**********************************************************************
int check_leftline_forC( void )
{
    int ret = 0;

    if(tsl1401_Wide > 28){
		  if(tsl1401_Center < -1){//センター右寄り
			  ret = 1;
		  }
	  }
    return ret;
}

//**********************************************************************
// ステアリング角度取得
// 引数　 なし
// 戻り値 ハンドル角度のA/D値
//**********************************************************************
int getServoAngle( void )
{
  return( (int)AD_023 - angle0_ad );
}

//**********************************************************************
// 坂角度取得
// 引数　 なし
// 戻り値 坂角度のA/D値
//**********************************************************************
int getSakaAngle( void )
{
  static int i_angle_x = 0;				/* ジャイロセンサーの値	*/
  static int i_pre_angle_x[2] = {0};		/* ジャイロセンサーの過去の値	*/

  i_pre_angle_x[0] = i_pre_angle_x[1];
	i_pre_angle_x[1] = i_angle_x;
	
	i_angle_x = (i_angle_x * 85/100) + (AD_008 * 15/100);
	
	if(((i_pre_angle_x[0] < i_pre_angle_x[1]) && (i_pre_angle_x[1] < i_angle_x))  || ((i_angle_x < i_pre_angle_x[1]) && (i_pre_angle_x[1] < i_pre_angle_x[0])))
		i_angle_x = i_pre_angle_x[1];
		
	else if(((i_pre_angle_x[1] < i_pre_angle_x[0]) && (i_pre_angle_x[0] < i_angle_x))  || ((i_angle_x < i_pre_angle_x[0]) && (i_pre_angle_x[0] < i_pre_angle_x[1])))
		i_angle_x = i_pre_angle_x[0];

  return i_angle_x;         // 坂のA/D値
}


//**********************************************************************
// ステアリングモータ PD制御
// 引数　 なし
// 戻り値 グローバル変数 pwm_trace に代入
//**********************************************************************
void servoControl( void )
{
    static int tsl1401_Center_before = 0;         // 前回のアナログセンサ値
    int ret;
    float work_p, work_d;

    // ステアリングモータ用PWM値計算
    work_p = df_p * (tsl1401_Center64 -64);                   // 比例
    work_d = df_d * (tsl1401_Center_before - (tsl1401_Center64 -64) );  // 微分(目安はPの5～10倍)
    ret = (int)(work_p - work_d);

    // PWMの上限、下限の設定
    if( ret >  100 ) {
        ret =  100;
    }
    if( ret < -100 ) {
        ret = -100;
    }
    pwm_trace = ret;

    tsl1401_Center_before = (tsl1401_Center64 -64);                 // 次回はこの値が1ms前の値となる
}

//**********************************************************************
// ステアリングモータ 角度のPD制御
// 引数　 なし
// 戻り値 グローバル変数 pwm_kakudo に代入
//**********************************************************************
void servoControl2( void )
{
  static int  angle_before;
  float work_p, work_d, kp, kd;

  kp = 0.625;                           // 比例定数
  kd = 3.125;                           // 微分定数(目安はPの5～10倍)

  // ステアリングモータ用PWM値計算
  work_p = kp * ( ang - set_angle );    // 比例
  work_d = kd * ( angle_before - ang ); // 微分
  int ret = work_p - work_d;

  // PWMの上限、下限の設定
  if( ret >  100 ) {
      ret =  100;
  }
  if( ret < -100 ) {
      ret = -100;
  }
  pwm_kakudo = ret;

  angle_before = ang;                   // 次回はこの値が1ms前の値となる
}

//**********************************************************************
// 外輪のPWMから、内輪のPWMを割り出す　ハンドル角度は現在の値を使用
// 引数　 外輪PWM
// 戻り値 内輪PWM
//**********************************************************************
int diff( int pwm )
{
  int i  = getServoAngle() / 48;        // 1度あたりの増分で割る
  if( i <  0 ) {
    i = -i;
  }
  if( i > 45 ) {
    i = 45;
  }

  int ret = revolution_difference[i] * pwm / 100;

  return ret;
}


//**********************************************************************
// char型データの値をlong型変数に2進数で変換
// 引数　 unsigned char 変換元の8bitデータ
// 戻り値 unsigned long 変換先の変数(0～11111111) ※0か1しかありません
//**********************************************************************
unsigned long convertBCD_CharToLong( unsigned char hex )
{
  unsigned long ret = 0;

  for( int i=0; i<8; i++ ) {
    ret *= 10;
    if( (hex & 0x80) != 0 ) {
      ret += 1;
    }
    hex <<= 1;
  }

  return ret;
}

//**********************************************************************
// モジュール名 getCompileYear
// 処理概要     コンパイルした時の年を取得
// 引数　       なし
// 戻り値       年
//**********************************************************************
int getCompileYear( const char *p )
{
  int i = atoi( p + 7 );

  if( i < 1980 || i > 2107 ) {
    i = 2024;
  }

  return i;
}

//**********************************************************************
// モジュール名 getCompileMonth
// 処理概要     コンパイルした時の月を取得
// 引数　       なし
// 戻り値       月
//**********************************************************************
int getCompileMonth( const char *p )
{
  const char monthStr[] = { "JanFebMarAprMayJunJulAugSepOctNovDec" };

  for( int i=0; i<12; i++ ) {
    int r = strncmp( monthStr + i * 3, p, 3 );
    if( r == 0 ) {
      return i + 1;
    }
  }

  return 1;
}

//**********************************************************************
// モジュール名 getCompileDay
// 処理概要     コンパイルした時の日を取得
// 引数　       なし
// 戻り値       日
//**********************************************************************
int getCompileDay( const char *p )
{
  int i = atoi( p + 4 );

  if( i < 1 || i > 31 ) {
    i = 1;
  }

  return i;
}

//**********************************************************************
// モジュール名 getCompileHour
// 処理概要     コンパイルした時の時を取得
// 引数　       なし
// 戻り値       時
//**********************************************************************
int getCompileHour( const char *p )
{
  int i = atoi( p );

  if( i < 0 || i > 23 ) {
    i = 0;
  }

  return i;
}

//**********************************************************************
// モジュール名 getCompilerMinute
// 処理概要     コンパイルした時の分を取得
// 引数　       なし
// 戻り値       分
//**********************************************************************
int getCompilerMinute( const char *p )
{
  int i = atoi( p + 3 );

  if( i < 0 || i > 59 ) {
    i = 0;
  }

  return i;
}

//**********************************************************************
// モジュール名 getCompilerSecond
// 処理概要     コンパイルした時の秒を取得
// 引数　       なし
// 戻り値       秒
//**********************************************************************
int getCompilerSecond( const char *p )
{
  int i = atoi( p + 6 );

  if( i < 0 || i > 59 ) {
    i = 0;
  }

  return i;
}

//**********************************************************************
// microSD.close時に、ファイルの日付・時刻を設定
//**********************************************************************
void set_microSD_dateTime( uint16_t* date, uint16_t* time )
{
  // FAT_DATEマクロでフィールドを埋めて日付を返す
  *date = FAT_DATE( (uint16_t)getCompileYear( C_DATE ),
                    (uint8_t)getCompileMonth( C_DATE ),
                    (uint8_t)getCompileDay( C_DATE ) );

  // FAT_TIMEマクロでフィールドを埋めて時間を返す
  *time = FAT_TIME( getCompileHour( C_TIME ),
                    getCompilerMinute( C_TIME ),
                    getCompilerSecond( C_TIME ) );
}

//**********************************************************************
// tsl1401 
//**********************************************************************
void tsl1401(){
 
  static int EXcnt = 0;
  
  if(EXcnt > 0){
    EXcnt--;
    return;
  }
  EXcnt = 2; // EXcntの回数無視 1ときは2msに1回撮影する
  

  switch(tsl1401_mode){
			case 0://通常モード
        ImageCapture(TSL1401_LineStart,TSL1401_LineStop);			//イメージキャプチャー
	      binarization(TSL1401_LineStart,TSL1401_LineStop); 		//２値化
	      WhiteLineWide(TSL1401_LineStart,TSL1401_LineStop);		//白ラインの測定

        break;

      case 1://坂モード
				ImageCapture(TSL1401_LineStartSaka,TSL1401_LineStopSaka);			//イメージキャプチャー
				binarization(TSL1401_LineStartSaka,TSL1401_LineStopSaka); 		//２値化
				WhiteLineWide(TSL1401_LineStartSaka,TSL1401_LineStopSaka);		//白ラインの測定
				
				break;
				
			case 2://右無視

				ImageCapture(TSL1401_LineStartNonR,TSL1401_LineStopNonR);			//イメージキャプチャー	
				binarization(TSL1401_LineStartNonR,TSL1401_LineStopNonR); 		//２値化
				WhiteLineWide(TSL1401_LineStartNonR,TSL1401_LineStopNonR);		//白ラインの測定
				
				break;
				
			case 3://左無視
        ImageCapture(TSL1401_LineStartNonL,TSL1401_LineStopNonL);			//イメージキャプチャー
				binarization(TSL1401_LineStartNonL,TSL1401_LineStopNonL); 		//２値化
				WhiteLineWide(TSL1401_LineStartNonL,TSL1401_LineStopNonL);		//白ラインの測定
			
				break;
  }

  tsl1401_Center_lasttime = tsl1401_Center64;//過去の値を保存

#ifdef DEBUG_PRINT
  static int print_cnt = 0;
  print_cnt++;
	if(print_cnt >= 100 / (EXcnt+1)){
  //if(print_cnt >= 100 ){
    for(int i = TSL1401_LineStart; i <=TSL1401_LineStop; i+=1)Serial.print(BinarizationData[i]);
/*
    for(int i = TSL1401_LineStart; i <=TSL1401_LineStop; i+=1){
      Serial.print(ImageData[i]);
      Serial.print(",");
    }
*/

    Serial.print(" Max2 = ");Serial.print(tsl1401_Max2);
    Serial.print(" Min2 = ");Serial.print(tsl1401_Min2);
    Serial.print(" Center = ");Serial.print(tsl1401_Center64);
    Serial.print(" Wide = ");Serial.print(tsl1401_Wide);
    Serial.print(" Lsensor = ");Serial.print(tsl1401_Lsensor);
    Serial.print(" Rsensor = ");Serial.print(tsl1401_Rsensor);
    Serial.print(" mode = ");Serial.print(tsl1401_mode);
    Serial.print(" WB_ave= ");Serial.print(tsl1401_WB_ave);
    Serial.println("");
    print_cnt = 0;
  }
#endif

}

 /************************************************************************/
/* イメージキャプチャ                                                   */
/************************************************************************/
void ImageCapture(int linestart, int linestop){	 
	
  unsigned char i;

	tsl1401_Max = 0;
	tsl1401_Max2 = 0;

	tsl1401_Min = 40900;
  tsl1401_Min2 = 40900;

	TSL1401_SI_HIGH;
	TSL1401_CLK_HIGH;
	TSL1401_SI_LOW;
	ImageData[0] = 0;
	TSL1401_CLK_LOW;
	for(i = 1; i < TSL1401_LineStart; i++) {		
		TSL1401_CLK_HIGH;	
    //メモ　処理高速化のためR8Cでは不要なAD変換はしていなかったがRAでは実施
    //      マイコンの性能が良いので CLK_HIGH と　CLK_LOW　の間に待ち時間が必要  ない場合数回に一回謎の値になる
    do{
		  ImageData[i] = AD_002 ;
    }while(ImageData[i] == 0);

		TSL1401_CLK_LOW;
	}
	for(i = TSL1401_LineStart; i < linestart; i++) {		
		TSL1401_CLK_HIGH;	
    do{
		  ImageData[i] = AD_002 ;
    }while(ImageData[i] == 0);
		TSL1401_CLK_LOW;
	}
	
	for(i = linestart; i <= linestop; i++) {				
		 
		TSL1401_CLK_HIGH;

		do{
		  ImageData[i] = AD_002 ; 
    }while(ImageData[i] == 0);

		TSL1401_CLK_LOW;
		
		if(tsl1401_Max2 < ImageData[i]){
			tsl1401_Max2 = ImageData[i];
			
			if(tsl1401_Max < ImageData[i]){
				tsl1401_Max2 = tsl1401_Max;
				tsl1401_Max = ImageData[i];
			}
			
		}
		
    if(tsl1401_Min2 > ImageData[i]){
			tsl1401_Min2 = ImageData[i];

      if(tsl1401_Min > ImageData[i]){
		  	tsl1401_Min2 = tsl1401_Min;
        tsl1401_Min = ImageData[i];
		  }
		}
		
	}
	
	for(i = linestop+1; i <= TSL1401_LineStop; i++) {		
		TSL1401_CLK_HIGH;
		do{
		  ImageData[i] = AD_002 ;
    }while(ImageData[i] == 0);
    
		TSL1401_CLK_LOW;
	}
	for(i = TSL1401_LineStop+1; i < 128; i++) {		
		TSL1401_CLK_HIGH;		
    do{
		  ImageData[i] = AD_002 ; 
    }while(ImageData[i] == 0);

		TSL1401_CLK_LOW;
	}
	
	TSL1401_CLK_HIGH;
	TSL1401_CLK_LOW;

}

/************************************************************************/
/* ２値化                                                               */
/************************************************************************/
void binarization(int linestart, int linestop)
{
	int i,a;
	
	/* 最高値と最低値から間の値を求める */

/*
	a  = ((tsl1401_Max2 + tsl1401_Min2) >> 1);
	tsl1401_Ave = ((a+tsl1401_Min2) >> 1);
	tsl1401_Ave = ((a+tsl1401_Ave) >> 1);
*/

  a  = ((tsl1401_Max2 + tsl1401_Min2) >> 1);
  tsl1401_Ave = ((a+tsl1401_Max2) >> 1);


 // tsl1401_Ave   = ((tsl1401_Max2 + tsl1401_Min2) >> 1);

	
	/* 黒は０　白は１にする */
	tsl1401_White = 0;					/* 白の数を０にする */
	
	if(tsl1401_Max2 - tsl1401_Min2 > 400){ //最大と最小の差があるとき
    for(i = linestart ; i <= linestop; i++) {
			if( ImageData[i] > tsl1401_Ave ){ //閾値以上
				tsl1401_White++;			
				BinarizationData[i] = 1;
			}else{
				BinarizationData[i] = 0;
			}	
		}

    if(tsl1401_White < 15){
      tsl1401_WB_ave = tsl1401_Ave;
    }
    
  
  }else{
    if(tsl1401_Min2 > tsl1401_WB_ave){
      /* 白が一直線のとき */
      tsl1401_White = 127;
      for(i = linestart ; i <= linestop; i++) {
        BinarizationData[i] = 1;
      }
    }else if(tsl1401_Max2 < tsl1401_WB_ave){
      /* 黒が一面のとき */
      for(i = linestart ; i <= linestop; i++) {
				  BinarizationData[i] = 0;
			}
    }else{
      //差は少ないが真っ黒、真っ白ではなさそう
      for(i = linestart ; i <= linestop; i++) {
        if( ImageData[i] > tsl1401_Ave ){ //閾値以上	
          tsl1401_White++;			
          BinarizationData[i] = 1;
        }else{
          BinarizationData[i] = 0;
        }	
      }
    }
	}

	//範囲外は黒に
	for(i = 0; i < linestart; i++){
		BinarizationData[i] = 0;
	}
	for(i = linestop+1; i < 128; i++){
		BinarizationData[i] = 0;
	}
}

/************************************************************************/
/* 白線の幅を測定                                                       */
/************************************************************************/
void WhiteLineWide(int linestart, int linestop)
{
	int t,i;
		
	tsl1401_Lsensor = linestart;
  tsl1401_Rsensor = linestop;
  tsl1401_Wide = 0;
  t = 0;		

	if(tsl1401_Center_lasttime < 64){//ライン左寄り
		for(i = linestart ; i <= linestop; i++) {
			if(t==0){
				if( BinarizationData[i] ){					/* 左から最初の白 */
					tsl1401_Lsensor = i;
					t = 1;
				}
			}else if(t==1){
				if( !BinarizationData[i] ){					/* 左から最初の黒 */			
					tsl1401_Rsensor = i;
					t = 2;
				}
			}
		}
	}else{//ライン右寄り
		for(i = linestop; i >= linestart; i--) {
			if(t==0){
				if( BinarizationData[i] ){					/* 右から最初の白 */
					tsl1401_Rsensor = i;
					t = 1;
				}
			}else if(t==1){
				if( !BinarizationData[i] ){					/* 右から最初の黒 */			
					tsl1401_Lsensor = i;
					t = 2;
				}
			}
		}	
	}
		
	
	if(tsl1401_White > 50){//全白にする
		tsl1401_Wide = 127;tsl1401_Center64 = 64;						/* 白一面 */
		
	}else if((tsl1401_White > 5) && ((linestop - linestart) > 4)){//白が少なすぎない && ラインを探す範囲が狭すぎない
	
		tsl1401_Wide = tsl1401_Rsensor - tsl1401_Lsensor;					/* 幅を求める */	
		tsl1401_Center64 = (tsl1401_Lsensor + tsl1401_Rsensor) >> 1;		/* 重心を求める */	
			
			
		//ライン細すぎ || ( 前回、黒又は白一色ではない && ハーフラインなどではない &&  (急にラインが移動した))
		if((((tsl1401_mode == 1) && (tsl1401_Wide < 4)) || ((tsl1401_mode != 1) && (tsl1401_Wide < 6))) || ((tsl1401_Center_lasttime != 64) && (tsl1401_White < 20) && (((tsl1401_Center64 - tsl1401_Center_lasttime) > 10) || ((tsl1401_Center64 - tsl1401_Center_lasttime) < -10)))){
					
			if(tsl1401_Center_lasttime < 64){
						
				WhiteLineWide(tsl1401_Rsensor,linestop);//もう一度ラインを探す			
			}else{
					
				WhiteLineWide(linestart,tsl1401_Lsensor);//もう一度ラインを探す
			}
		}	
	}else{
    //全黒にする
		tsl1401_Wide = 0;tsl1401_Center64 = 64;						/* 黒一面 */
	}	

  tsl1401_Center = tsl1401_Center64 -64;			
}

//**********************************************************************
// end of file
//**********************************************************************

/*
Ver.1.00 2024.07.07 作成
Ver.1.01 2024.08.09 getAnalogSensor関数のデジタルセンサの状態を追加
*/
