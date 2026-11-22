#!/usr/bin/env python3
import pathlib, subprocess, tempfile
root=pathlib.Path(__file__).resolve().parents[2]
with tempfile.TemporaryDirectory(prefix="sentry-vision-test-") as d:
    exe=pathlib.Path(d)/"check"
    subprocess.run(["cc","-std=c11","-Wall","-Wextra","-Werror","-I"+str(root/"application/sentry"),str(root/"scripts/diagnostics/sentry_vision_test.c"),str(root/"application/sentry/sentry_vision.c"),"-lm","-o",str(exe)],check=True)
    subprocess.run([str(exe)],check=True)

other=root.parent/("nyush_rm_sentry_beyblading" if root.name=="nyush_rm_sentry" else "nyush_rm_sentry")
if other.exists():
    for name in ("sentry_vision.c", "sentry_vision.h", "sentry_vision_protocol.h"):
        assert (root/"application/sentry"/name).read_bytes()==(other/"application/sentry"/name).read_bytes(), "firmware helper copies differ: "+name

# Compile the actual legacy UART/VCP validation and stream bodies with tiny I/O
# doubles; this checks production functions rather than a Python codec copy.
source=(root/"modules/master_machine/master_process.c").read_text()
def function(name):
    import re
    match=re.search(r"static\s+(?:uint8_t|void)\s+"+name+r"\s*\([^)]*\)\s*\{",source)
    assert match, "missing production helper "+name
    begin=match.start(); pos=source.index("{",match.start()); depth=1; end=pos+1
    while depth:
        depth += (source[end]=="{")-(source[end]=="}");end+=1
    return source[begin:end]
preamble=r"""
#include "sentry_vision.h"
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <string.h>
#define VISION_RECV_SIZE 29u
#define VISION_SP_HEADER_1 'S'
#define VISION_SP_HEADER_2 'P'
typedef struct __attribute__((packed)) {uint8_t head[2],mode;float yaw,yaw_vel,yaw_acc,pitch,pitch_vel,pitch_acc;uint16_t crc16;} VisionToGimbal_s;
static unsigned applied;
static void *vision_daemon_instance;
static void DaemonReload(void *p){(void)p;}
static void ApplyVisionPacket(const VisionToGimbal_s *p){assert(p->mode<=2);applied++;}
static uint16_t crc_16(uint8_t *p,size_t n){return SentryVisionCrc(p,n);}
"""
main=r"""
int main(void){
 VisionToGimbal_s p={{'S','P'},1,.1f,0,0,0,0,0,0};
 p.crc16=crc_16((uint8_t*)&p,27);assert(ValidateVisionDataFast(&p));
 for(unsigned split=0;split<=29;split++){
  unsigned before=applied;DecodeVisionBytes((uint8_t*)&p,split);DecodeVisionBytes(((uint8_t*)&p)+split,29-split);assert(applied==before+1);
 }
 p.yaw=NAN;p.crc16=crc_16((uint8_t*)&p,27);assert(!ValidateVisionDataFast(&p));
 p.yaw=0;p.yaw_vel=101;p.crc16=crc_16((uint8_t*)&p,27);assert(!ValidateVisionDataFast(&p));
 p.yaw_vel=0;p.pitch_acc=INFINITY;p.crc16=crc_16((uint8_t*)&p,27);assert(!ValidateVisionDataFast(&p));
 p.pitch_acc=1001;p.crc16=crc_16((uint8_t*)&p,27);assert(!ValidateVisionDataFast(&p));
 p.pitch_acc=0;p.crc16=crc_16((uint8_t*)&p,27);uint8_t stream[60]={0x53,0x53};memcpy(stream+2,&p,29);memcpy(stream+31,&p,29);
 unsigned before=applied;DecodeVisionBytes(stream,60);assert(applied==before+2);return 0;
}
"""
with tempfile.TemporaryDirectory(prefix="sentry-legacy-sp-") as d:
    test=pathlib.Path(d)/"legacy.c";exe=pathlib.Path(d)/"legacy"
    test.write_text(preamble+function("ValidateVisionPayload")+function("ValidateVisionDataFast")+function("DecodeVisionBytes")+main)
    subprocess.run(["cc","-std=c11","-Wall","-Wextra","-Werror","-I"+str(root/"application/sentry"),str(test),str(root/"application/sentry/sentry_vision.c"),"-lm","-o",str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
print("legacy SP production validation/fragmentation: PASS")

bsp_source=(root/"bsp/usart/bsp_usart.c").read_text()
start=bsp_source.index("void HAL_UARTEx_RxEventCallback(");pos=bsp_source.index("{",start);end=pos+1;depth=1
while depth:
    depth+=(bsp_source[end]=="{")-(bsp_source[end]=="}");end+=1
callback=bsp_source[start:end]
bsp_stubs=r"""
#include "bsp_usart.h"
static USARTInstance instance;
static USARTInstance *usart_instance[1]={&instance};
static uint8_t idx=1;
#define DMA_IT_HT 0
#define __HAL_DMA_DISABLE_IT(a,b) ((void)(a),(void)(b))
static void HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *a,uint8_t *b,uint16_t c){(void)a;(void)b;(void)c;}
static void Receive(void){DecodeVisionBytes(instance.recv_buff,instance.recv_len);}
"""
bsp_main=r"""
int main(void){
 UART_HandleTypeDef uart={0};instance.usart_handle=&uart;instance.module_callback=Receive;instance.recv_buff_size=29;
 VisionToGimbal_s p={{'S','P'},1,.1f,0,0,0,0,0,0};p.crc16=crc_16((uint8_t*)&p,27);
 memcpy(instance.recv_buff,&p,5);HAL_UARTEx_RxEventCallback(&uart,5);assert(instance.recv_len==5&&applied==0);
 memcpy(instance.recv_buff,((uint8_t*)&p)+5,24);HAL_UARTEx_RxEventCallback(&uart,24);assert(instance.recv_len==24&&applied==1);
 return 0;
}
"""
with tempfile.TemporaryDirectory(prefix="sentry-uart-length-") as d:
    directory=pathlib.Path(d);test=directory/"uart.c";exe=directory/"uart"
    (directory/"main.h").write_text("typedef struct {void *hdmarx;} UART_HandleTypeDef;\n")
    test.write_text(preamble+function("ValidateVisionPayload")+function("ValidateVisionDataFast")+function("DecodeVisionBytes")+bsp_stubs+callback+bsp_main)
    subprocess.run(["cc","-std=c11","-Wall","-Wextra","-Werror","-I"+d,"-I"+str(root/"bsp/usart"),"-I"+str(root/"application/sentry"),str(test),str(root/"application/sentry/sentry_vision.c"),"-lm","-o",str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
print("production UART short-read length propagation: PASS")

if root.name == "nyush_rm_sentry_beyblading":
    # Compile the actual vision branch of UpdateAimYawMotor with the real robot
    # calibration and encoder-coordinate helper. Motor/RTOS dependencies are
    # replaced only outside this adapter branch.
    adapter_source=(root/"application/sentry/sentry_steer_nav2.c").read_text()
    start=adapter_source.index("    if (vision_state.active)",adapter_source.index("static void UpdateAimYawMotor("))
    pos=adapter_source.index("{",start);end=pos+1;depth=1
    while depth:
        depth+=(adapter_source[end]=="{")-(adapter_source[end]=="}");end+=1
    branch=adapter_source[start:end]
    define_begin=adapter_source.index("#define AIM_YAW_SP_COMMAND_SIGN")
    define_end=adapter_source.index("#if",define_begin)
    definitions=adapter_source[define_begin:define_end]
    adapter_preamble=r"""
#include "sentry_vision.h"
#include "sentry_aim_yaw_limit.h"
#include "robot_sentry_swerve.h"
#include <assert.h>
#include <math.h>
#include <string.h>
#define portTICK_PERIOD_MS 1u
static uint32_t xTaskGetTickCount(void){return 100u;}
static SentryVisionState_s vision_state;
static SentryVisionFeedback_s vision_feedback;
"""
    wrapper=r"""
static float Adapter(uint16_t ecd,int16_t manual,uint8_t *was_enabled){
 const float relative=SentryYawRelativeDeg(ecd,SENTRY_AIM_YAW_RIGHT_LIMIT_ECD);
 uint8_t enabled=1u;
 float motor_rate=(float)manual*(float)SENTRY_AIM_YAW_COMMAND_SIGN;
"""+branch+r"""
 *was_enabled=enabled;return motor_rate;
}
static void Prepare(float target){
 memset(&vision_state,0,sizeof(vision_state));memset(&vision_feedback,0,sizeof(vision_feedback));
 vision_state.active=1;vision_state.seen=1;vision_state.flags=SENTRY_VISION_ACTIVE;
 vision_state.command.mode=1;vision_state.command.yaw=target;vision_state.command_ms=100;
}
int main(void){
 const uint16_t middle=(SENTRY_AIM_YAW_RIGHT_LIMIT_ECD+8192u-1365u)%8192u;
 const float relative=SentryYawRelativeDeg(middle,SENTRY_AIM_YAW_RIGHT_LIMIT_ECD);
 assert(relative<-59&&relative>-61);uint8_t enabled;
 Prepare(0);float rate=Adapter(middle,0,&enabled);assert(enabled&&vision_state.active&&fabsf(rate)<1e-6f);
 Prepare(.1f);rate=Adapter(middle,0,&enabled);assert(enabled&&vision_state.active&&rate>0); /* SP positive yaw is RIGHT, encoder increases RIGHT. */
 assert(fabsf(rate-400)<.01f);
 Prepare(-.1f);rate=Adapter(middle,0,&enabled);assert(enabled&&rate<0&&fabsf(rate+400)<.01f);
 const uint16_t right=(SENTRY_AIM_YAW_RIGHT_LIMIT_ECD+8192u-SENTRY_AIM_YAW_RIGHT_GUARD_ECD)%8192u;
 Prepare(.1f);rate=Adapter(right,0,&enabled);assert(!enabled&&!vision_state.active&&rate==0);
 const uint16_t left=(SENTRY_AIM_YAW_RIGHT_LIMIT_ECD+8192u-(SENTRY_AIM_YAW_TRAVEL_ECD-SENTRY_AIM_YAW_LEFT_GUARD_ECD))%8192u;
 Prepare(-.1f);rate=Adapter(left,0,&enabled);assert(!enabled&&!vision_state.active&&rate==0);
 Prepare(0);vision_state.active=0;rate=Adapter(middle,100,&enabled);assert(enabled&&rate==-100); /* Existing manual positive LEFT stays unchanged. */
 return 0;
}
"""
    with tempfile.TemporaryDirectory(prefix="sentry-yaw-adapter-") as d:
        test=pathlib.Path(d)/"adapter.c";exe=pathlib.Path(d)/"adapter"
        test.write_text(adapter_preamble+definitions+wrapper)
        subprocess.run(["cc","-std=c11","-Wall","-Wextra","-Werror","-I"+str(root/"application/sentry"),"-I"+str(root/"application/robot_configs"),str(test),str(root/"application/sentry/sentry_vision.c"),"-lm","-o",str(exe)],check=True)
        subprocess.run([str(exe)],check=True)
    print("production guarded SP yaw adapter/calibration: PASS")

# Exercise the actual telemetry builder and legacy Euler helper with only the
# gyro and CMSIS trig calls stubbed; internal yaw may legitimately be multi-turn.
imu_source=(root/"modules/imu/ins_task.c").read_text()
start=imu_source.index("void EularAngleToQuaternion(");pos=imu_source.index("{",start);end=pos+1;depth=1
while depth:
    depth+=(imu_source[end]=="{")-(imu_source[end]=="}");end+=1
phase_preamble=r"""
#include <assert.h>
#include <math.h>
#include <string.h>
#define VISION_DEG_TO_RAD .01745329251f
#define GIMBAL_YAW_CTRL_FROM_IMU_SIGN 1
#define GYRO2GIMBAL_DIR_PITCH 1
#define arm_cos_f32 cosf
#define arm_sin_f32 sinf
typedef struct {float q[4],yaw,yaw_vel,pitch,pitch_vel;} GimbalToVision_s;
static struct {float q[4],yaw,pitch,roll,yaw_rad,yaw_vel_rad_s,pitch_rad,pitch_vel_rad_s;} send_data;
static float gyro[3]={.1f,.2f,.3f};
static float *INS_GetGyro(void){return gyro;}
"""
phase_main=r"""
int main(void){
 GimbalToVision_s packet={0};send_data.yaw=721;send_data.pitch=17;send_data.roll=23;
 FillGimbalToVisionAttitude(&packet);
 assert(fabsf(packet.yaw-VISION_DEG_TO_RAD)<3e-6f);
 assert(send_data.yaw==721&&send_data.pitch==17&&send_data.roll==23);
 float expected[4];EularAngleToQuaternion(1,17,23,expected);float dot=0;
 for(unsigned i=0;i<4;i++){dot+=packet.q[i]*expected[i];}assert(fabsf(dot)>.999999f);
 assert(fabsf(packet.pitch-17*VISION_DEG_TO_RAD)<1e-6f);
 assert(packet.yaw_vel==.3f&&packet.pitch_vel==.1f);
 send_data.yaw=-721;FillGimbalToVisionAttitude(&packet);assert(fabsf(packet.yaw+VISION_DEG_TO_RAD)<3e-6f);
 send_data.yaw=359;FillGimbalToVisionAttitude(&packet);assert(fabsf(packet.yaw+VISION_DEG_TO_RAD)<3e-6f);
 send_data.pitch=400;FillGimbalToVisionAttitude(&packet);assert(packet.pitch>3.15f); /* Invalid pitch is never disguised by wrapping. */
 return 0;
}
"""
with tempfile.TemporaryDirectory(prefix="sentry-telemetry-phase-") as d:
    test=pathlib.Path(d)/"phase.c";exe=pathlib.Path(d)/"phase"
    test.write_text(phase_preamble+imu_source[start:end]+function("FillGimbalToVisionAttitude")+phase_main)
    subprocess.run(["cc","-std=c11","-Wall","-Wextra","-Werror",str(test),"-lm","-o",str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
print("production telemetry multi-turn yaw phase/quaternion: PASS")
