#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Bool,Int32
from pathlib import Path
import sounddevice as sd
import sherpa_onnx
import numpy as np
import scipy.signal as signal
from state_machine_msg.msg import keywords
from unitree_go.msg import WirelessController
import subprocess
import os
import re


class KeywordSpotterNode:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node("keyword_spotter", anonymous=True)

        # Load parameters from ROS parameter server
        self.tokens = rospy.get_param("~tokens", "")
        self.encoder = rospy.get_param("~encoder", "")
        self.decoder = rospy.get_param("~decoder", "")
        self.joiner = rospy.get_param("~joiner", "")
        self.keywords_file = rospy.get_param("~keywords_file", "")
        self.num_threads = rospy.get_param("~num_threads", 1)
        self.provider = rospy.get_param("~provider", "cpu")
        self.max_active_paths = rospy.get_param("~max_active_paths", 4)
        self.keywords_score = rospy.get_param("~keywords_score", 1.0)
        self.keywords_threshold = rospy.get_param("~keywords_threshold", 0.25)
        self.num_trailing_blanks = rospy.get_param("~num_trailing_blanks", 1)
        
        # USB麦克风通常是48000Hz，模型通常需要16000Hz
        self.model_sample_rate = 16000  # 模型需要的采样率
        self.device_sample_rate = rospy.get_param("~device_sample_rate", 48000)  # 设备采样率
        self.sample_rate = self.device_sample_rate  # 使用设备采样率进行录音
        self.samples_per_read = int(0.1 * self.sample_rate)  # 0.1 second = 100ms

        # State to control audio input
        self.audio_paused = False  # Default is not paused

        # 添加激活状态控制
        self.awaiting_activation = True  # 初始状态：等待"你好小益"激活
        self.listening_for_command = False  # 是否正在监听命令
        self.command_timer = None  # 命令监听计时器
        self.command_timeout = 10.0  # 命令接收超时时间（秒）
        
        # 存储选择的音频设备ID
        self.selected_input_device_id = None  # 输入设备ID
        self.selected_output_device_id = None  # 输出设备ID
        self.audio_stream = None

        

        # Check that required files exist
        self.assert_file_exists(self.tokens, "tokens")
        self.assert_file_exists(self.encoder, "encoder")
        self.assert_file_exists(self.decoder, "decoder")
        self.assert_file_exists(self.joiner, "joiner")
        self.assert_file_exists(self.keywords_file, "keywords_file")

        # Initialize the keyword spotter
        self.keyword_spotter = sherpa_onnx.KeywordSpotter(
            tokens=self.tokens,
            encoder=self.encoder,
            decoder=self.decoder,
            joiner=self.joiner,
            num_threads=self.num_threads,
            max_active_paths=self.max_active_paths,
            keywords_file=self.keywords_file,
            keywords_score=self.keywords_score,
            keywords_threshold=self.keywords_threshold,
            num_trailing_blanks=self.num_trailing_blanks,
            provider=self.provider,
        )
        self.stream = self.keyword_spotter.create_stream()

        # Publisher for keyword detection results
        # self.result_pub = rospy.Publisher("keyword_result", String, queue_size=10)
        self.keywords_command_pub = rospy.Publisher("keywords_command", keywords, queue_size=10)

        # Subscriber for controlling audio input
        rospy.Subscriber("audio_playing", Bool, self.audio_playing_callback)  # 语音播放暂停
        rospy.Subscriber("init_flag", Bool, self.initialize_callback)
        rospy.Subscriber("obs_flag", Bool, self.obs_callback)
        rospy.Subscriber("go2_control_status", Int32, self.go2_control_status_callback)
        rospy.Subscriber("nav_set_point", Int32, self.nav_set_point_callback)
        # rospy.Subscriber("/wireless_controller", WirelessController, self.wireless_controller_callback)
        

        # 创建定时器，每2秒检查并重置一次音频设备，防止系统修改
        self.device_timer = rospy.Timer(rospy.Duration(2), self.device_timer_callback)
        

        rospy.loginfo("Keyword Spotter Node initialized successfully.")

    def command_timer_callback(self, event):
        """命令接收超时回调函数"""
        if self.listening_for_command:
            rospy.loginfo("命令接收超时，返回等待激活状态")
            self.listening_for_command = False
            self.awaiting_activation = True
            # self.play_audio("timeout")  # 超时提示音

    def assert_file_exists(self, filepath, param_name):
        """Check if a file exists."""
        if not Path(filepath).is_file():
            rospy.logerr(f"Required file for parameter '{param_name}' does not exist: {filepath}")
            rospy.signal_shutdown(f"Missing required file: {filepath}")

    def audio_playing_callback(self, msg):
        """Callback function to handle audio_playing topic."""
        self.audio_paused = msg.data  # Update the audio paused state
        state = "paused" if self.audio_paused else "resumed"
        rospy.loginfo(f"Audio input {state} based on audio_playing topic.")

    def initialize_callback(self, msg):
        """Callback function to handle initialization flag."""
        if msg.data:
            rospy.loginfo("初始化成功")
            self.play_audio("初始化成功")

        else:
            rospy.loginfo("初始化标志未设置")

    def nav_set_point_callback(self, msg):
        """Callback function to handle navigation set point."""
        rospy.loginfo(f"接收到导航设置点: {msg.data}")
        if msg.data == 1:
            rospy.loginfo("标记为一号点位")
            self.play_audio("标记为一号点位")
        elif msg.data == 2:
            rospy.loginfo("标记为二号点位")
            self.play_audio("标记为二号点位")
        elif msg.data == 3:
            rospy.loginfo("标记为三号点位")
            self.play_audio("标记为三号点位")
        elif msg.data == 4:
            rospy.loginfo("标记为四号点位")
            self.play_audio("标记为四号点位")
        elif msg.data == 5:
            rospy.loginfo("标记为五号点位")
            self.play_audio("标记为五号点位")
        elif msg.data == 6:
            rospy.loginfo("标记为六号点位")
            self.play_audio("标记为六号点位")
        elif msg.data == 7:
            rospy.loginfo("标记为七号点位")
            self.play_audio("标记为七号点位")
        elif msg.data == 8:
            rospy.loginfo("标记为八号点位")
            self.play_audio("标记为八号点位")
        elif msg.data == 9:
            rospy.loginfo("标记为九号点位")
            self.play_audio("标记为九号点位")

    def go2_control_status_callback(self, msg):
        """Callback function to handle Go2 control status."""
        if msg.data == 1:
            rospy.loginfo("遥控模式")
            self.play_audio("遥控模式")
        elif msg.data == 0:
            rospy.loginfo("导航模式")
            self.play_audio("导航模式")

    def obs_callback(self, msg):
        """Callback function to handle obstacle avoidance flag."""
        if msg.data:
            rospy.loginfo("触发避障碍")
            self.play_audio("避障")
        else:
            rospy.loginfo("未触发避障碍")
        
    
    # def wireless_controller_callback(self, msg):
    #     """处理无线控制器消息，根据keys值播放对应视频"""
    #     try:
    #         # 定义keys值与视频文件的映射
    #         key_to_video = {
    #             2048: "1",
    #             512: "2", 
    #             256: "3",
    #             1024: "4",
    #             4096:"5",
    #             8192:"6",
    #             16384:"7",
    #             32768:"8"
    #         }
            
    #         # 检查是否为目标按键
    #         if msg.keys in key_to_video:
    #             video_file = key_to_video[msg.keys]
    #             rospy.loginfo(f"检测到按键 {msg.keys}，播放视频: {video_file}")
    #             self.play_audio(video_file)
    #         else:
    #             rospy.logdebug(f"收到按键 {msg.keys}，无对应视频")
                
    #     except Exception as e:
    #         rospy.logerr(f"处理无线控制器消息时出错: {e}")
        
    def device_timer_callback(self, event):
        """定时器回调函数，定期检查并重置音频设备"""
        # 检查输入设备
        if not self.audio_stream or not self.audio_stream.active:
            # 如果音频流不存在或不活动，尝试重新设置
            rospy.loginfo("定时检查: 音频输入设备需要重置")
            self.setup_audio_stream()
        else:
            # 音频流正常，只是打印一个调试信息
            if hasattr(self, 'selected_input_device_id') and self.selected_input_device_id is not None:
                rospy.logdebug("定时检查: 音频输入设备正常, 设备ID: %s", self.selected_input_device_id)
        
        # 检查并设置输出设备
        self.set_default_output_device()
                
    def set_default_output_device(self):
        """设置默认的音频输出设备"""
        try:
            # 获取可用音频设备
            devices = sd.query_devices()
            output_device_id = None
            
            # 如果已经选择了一个输出设备，并且它仍然有效，就继续使用它
            if self.selected_output_device_id is not None:
                try:
                    if isinstance(self.selected_output_device_id, int) and 0 <= self.selected_output_device_id < len(devices):
                        device_info = sd.query_devices(self.selected_output_device_id)
                        if device_info['max_output_channels'] > 0:
                            output_device_id = self.selected_output_device_id
                            # 不要每次都打印，以减少日志冗余
                            rospy.logdebug(f"继续使用之前选择的输出设备: [{self.selected_output_device_id}] {device_info['name']}")
                except Exception as e:
                    rospy.logwarn(f"之前选择的输出设备不再可用: {e}")
            
            # 如果之前选择的设备无效，查找新的设备
            if output_device_id is None:
                # 优先查找内建输出设备或HDMI输出
                for i, dev in enumerate(devices):
                    if dev['max_output_channels'] > 0 and any(keyword in dev['name'].lower() for keyword in ["built-in", "hdmi", "analog", "speaker"]):
                        output_device_id = i
                        rospy.loginfo(f"选择输出设备: [{i}] {dev['name']}")
                        break
                
                # 如果没有找到特定类型的设备，选择任何有输出通道的设备
                if output_device_id is None:
                    for i, dev in enumerate(devices):
                        if dev['max_output_channels'] > 0:
                            output_device_id = i
                            rospy.loginfo(f"选择输出设备: [{i}] {dev['name']}")
                            break
            
            # 如果找到了有效的输出设备，设置为默认设备
            if output_device_id is not None:
                try:
                    # 获取当前默认输出设备ID
                    current_default = sd.default.device[1]
                    
                    # 如果当前默认设备与我们想要设置的不同，则进行设置
                    if current_default != output_device_id:
                        rospy.loginfo(f"更改默认输出设备从 {current_default} 到 {output_device_id}")
                        sd.default.device = (sd.default.device[0], output_device_id)
                        self.selected_output_device_id = output_device_id
                    else:
                        # 确保更新存储的输出设备ID
                        self.selected_output_device_id = output_device_id
                        rospy.logdebug(f"默认输出设备已经正确设置为 {output_device_id}")
                except Exception as e:
                    rospy.logwarn(f"设置默认输出设备时出错: {e}")
            else:
                rospy.logwarn("未找到合适的音频输出设备")
        
        except Exception as e:
            rospy.logerr(f"设置默认输出设备失败: {e}")
            
    def setup_audio_stream(self):
        """设置音频输入流"""
        # 关闭现有的音频流
        if hasattr(self, 'audio_stream') and self.audio_stream and self.audio_stream.active:
            try:
                self.audio_stream.stop()
                self.audio_stream.close()
                rospy.loginfo("关闭了现有的音频流")
            except Exception as e:
                rospy.logwarn(f"关闭音频流时出错: {e}")
        
        # 查找合适的音频设备ID
        device_id = self.find_audio_device()
        
        if device_id is not None:
            try:
                rospy.loginfo(f"设置音频输入流，设备ID: {device_id}，采样率: {self.sample_rate} Hz")
                self.audio_stream = sd.InputStream(
                    device=device_id,
                    channels=1,
                    dtype="float32",
                    samplerate=self.sample_rate,
                    callback=self.audio_callback,
                    blocksize=self.samples_per_read
                )
                self.audio_stream.start()
                self.selected_input_device_id = device_id
                rospy.loginfo("音频输入流已启动，正在监听关键词...")
                return True
            except Exception as e:
                rospy.logerr(f"启动音频输入流失败: {e}")
                return False
        else:
            rospy.logerr("未找到合适的音频输入设备")
            return False
            
    def find_audio_device(self):
        """查找合适的音频输入设备"""
        try:
            # 获取可用音频设备信息
            devices = sd.query_devices()
            rospy.loginfo("可用音频设备:")
            for i, dev in enumerate(devices):
                rospy.loginfo(f"[{i}] {dev['name']}: 输入通道 {dev['max_input_channels']}, 输出通道 {dev['max_output_channels']}")
            
            # 如果已经有选定的设备ID并且它仍然有效，则继续使用它
            if self.selected_input_device_id is not None:
                try:
                    if isinstance(self.selected_input_device_id, int) and 0 <= self.selected_input_device_id < len(devices):
                        device_info = sd.query_devices(self.selected_input_device_id)
                        if device_info['max_input_channels'] > 0:
                            rospy.loginfo(f"继续使用之前选择的输入设备: [{self.selected_input_device_id}] {device_info['name']}")
                            return self.selected_input_device_id
                except Exception as e:
                    rospy.logwarn(f"之前选择的输入设备不再可用: {e}")
            
            # 直接设置使用USB麦克风设备
            device_id = None
            
            # 查找USB Composite Device设备
            for i, dev in enumerate(devices):
                if "usb composite device" in dev['name'].lower() and dev['max_input_channels'] > 0:
                    device_id = i
                    rospy.loginfo(f"直接选择USB Composite Device麦克风设备: [{i}] {dev['name']}")
                    break
            
            # 如果没找到，再查找Jieli Technology USB设备
            if device_id is None:
                for i, dev in enumerate(devices):
                    if "jieli" in dev['name'].lower() and dev['max_input_channels'] > 0:
                        device_id = i
                        rospy.loginfo(f"直接选择Jieli USB麦克风设备: [{i}] {dev['name']}")
                        break
                    
            # 初始化device_name变量，避免引用前未定义的错误
            device_name = rospy.get_param("~device_name", "")
            
            # 如果找不到Jieli设备，查找任何USB麦克风
            if device_id is None:
                for i, dev in enumerate(devices):
                    if "usb" in dev['name'].lower() and dev['max_input_channels'] > 0:
                        device_id = i
                        rospy.loginfo(f"直接选择USB麦克风设备: [{i}] {dev['name']}")
                        break
            
            # 如果还是没找到，才尝试使用参数指定的设备或默认设备
            if device_id is None:
                # 尝试获取默认输入设备
                try:
                    default_device = sd.query_devices(kind='input')
                    rospy.loginfo(f"默认输入设备: {default_device['name']}")
                    default_device_id = sd.default.device[0]
                    rospy.loginfo(f"默认输入设备ID: {default_device_id}")
                except Exception as e:
                    rospy.logwarn(f"无法获取默认输入设备: {e}")
                    default_device_id = None
                
                # 允许通过参数指定设备 ID
                device_id = rospy.get_param("~device_id", default_device_id)
            
            # 如果指定了设备名称，尝试查找匹配的设备
            if device_name:
                found = False
                rospy.loginfo(f"尝试查找名称包含 '{device_name}' 的设备")
                for i, dev in enumerate(devices):
                    if device_name.lower() in dev['name'].lower() and dev['max_input_channels'] > 0:
                        device_id = i
                        rospy.loginfo(f"找到匹配的设备: [{i}] {dev['name']}")
                        found = True
                        break
                if not found:
                    rospy.logwarn(f"未找到名称包含 '{device_name}' 的设备，尝试使用其他方法选择设备")
            
            # 如果指定了设备ID
            if device_id is not None:
                rospy.loginfo(f"使用设备 ID: {device_id}")
                # 验证设备ID是否有效
                try:
                    if isinstance(device_id, int) and 0 <= device_id < len(devices):
                        rospy.loginfo(f"已选择设备: {devices[device_id]['name']}")
                        return device_id
                    else:
                        rospy.logwarn(f"提供的设备ID {device_id} 可能无效")
                except Exception as e:
                    rospy.logwarn(f"验证设备ID时出错: {e}")
            else:
                rospy.logwarn("未指定设备ID，尝试查找USB麦克风设备")
                # 首先尝试查找USB麦克风设备
                for i, dev in enumerate(devices):
                    if ("usb" in dev['name'].lower() or "jieli" in dev['name'].lower()) and dev['max_input_channels'] > 0:
                        device_id = i
                        rospy.loginfo(f"自动选择USB麦克风设备: [{i}] {dev['name']}")
                        return device_id
                
                # 如果没有找到USB麦克风，尝试查找ALSA捕获设备
                if device_id is None:
                    for i, dev in enumerate(devices):
                        if "alsa" in dev['name'].lower() and dev['max_input_channels'] > 0:
                            device_id = i
                            rospy.loginfo(f"自动选择ALSA设备: [{i}] {dev['name']}")
                            return device_id
            
            return device_id
            
        except Exception as e:
            rospy.logerr(f"查找音频设备失败: {e}")
            return None

    def audio_callback(self, indata, frames, time, status):
        """Callback function for processing audio input."""
        if status:
            rospy.logwarn(f"Audio input error: {status}")
        
        # 添加调试信息，检查是否收到音频数据
        # if indata is not None:
        #     max_value = abs(indata).max()
        #     if max_value > 0.01:  # 设置一个小的阈值来检测有意义的音频输入
        #         rospy.loginfo(f"接收到音频信号，最大振幅: {max_value:.4f}")

        # If audio is paused, skip processing
        if self.audio_paused:
            return

        # Process the audio input
        audio_data = indata.reshape(-1)  # Flatten the audio data
        
        # 如果设备采样率和模型采样率不同，需要进行重采样
        if self.device_sample_rate != self.model_sample_rate:
            import scipy.signal as signal
            # 使用scipy的resample函数进行重采样
            num_samples = int(len(audio_data) * self.model_sample_rate / self.device_sample_rate)
            audio_data = signal.resample(audio_data, num_samples)
        
        self.stream.accept_waveform(self.model_sample_rate, audio_data)

        # Perform decoding
        while self.keyword_spotter.is_ready(self.stream):
            self.keyword_spotter.decode_stream(self.stream)

        # Get keyword spotting result
        result = self.keyword_spotter.get_result(self.stream)
        if result:
            # 根据当前状态处理结果
            if result == "你好小益":
                # 收到激活词，开始监听命令
                rospy.loginfo("收到激活词'你好小益'，开始接收命令，10秒内有效")
                self.play_audio(result)
                self.awaiting_activation = False
                self.listening_for_command = True
                
                # 启动10秒计时器
                if self.command_timer:
                    self.command_timer.shutdown()
                self.command_timer = rospy.Timer(rospy.Duration(self.command_timeout), 
                                            self.command_timer_callback, oneshot=True)
            
            elif self.listening_for_command:
                # 已处于命令监听状态，接收到命令
                self.play_audio(result)
                msg = keywords()
                msg.command = result
                msg.target_point = 0    #3D导航版本不需要target_point
                # msg.target_point = self.solve_keywords_id(result)
                self.keywords_command_pub.publish(msg)
                
                # 记录结果
                rospy.loginfo(f"接收到命令: {result}")
                rospy.loginfo(f"命令ID: {msg.target_point}")
                
                # 接收到命令后，回到等待激活状态
                self.listening_for_command = False
                self.awaiting_activation = True
                
                # 关闭计时器
                if self.command_timer:
                    self.command_timer.shutdown()
                    self.command_timer = None
            
            else:
                # 未激活状态下收到非激活词，忽略
                rospy.loginfo(f"未激活状态下检测到关键词: {result}，请先说'你好小益'激活")
    
    def solve_keywords_id(self, keyword):
        """
        根据识别到的关键词映射到目标点编号
        
        参数:
            keyword (str): 识别到的关键词，如"欢迎"、"家务"等
            
        返回:
            int: 目标点的编号，如果无法匹配则返回0
        """
        # 关键词到目标点的映射
        keyword_to_point = {
            "迎接": 0,
            "家务": 1,
            "餐桌": 2,
            "客厅": 3,
            "卧室": 4,
            "暗黑": 5,
            "阅览": 6,
            "视觉": 7,
            "听觉": 8
        }
        
        # 遍历所有关键词，查找匹配项
        for key, point_id in keyword_to_point.items():
            if key in keyword:
                rospy.loginfo(f"关键词 '{keyword}' 匹配到目标点 {point_id}")
                return point_id
        

        
        rospy.logwarn(f"无法从关键词 '{keyword}' 解析出目标点编号")
        return 100
    
    def play_audio(self, keyword):
        """
        根据识别到的关键词播放相应的语音文件
        
        参数:
            keyword (str): 识别到的关键词，如"去一号点"
        """
        
        # 语音文件目录
        audio_dir = rospy.get_param("~audio_dir", os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "audio"))
        
        # 关键词到音频文件的映射
        keyword_to_audio = {
            # 导航命令
            "导航开始": "start_nav.wav",
            "导航终止": "cancel_nav.wav",
            
            # 方向命令
            # "前进": "forward.wav",
            "后退": "receive.wav",
            "左转": "receive.wav",
            "右转": "receive.wav",
            "停下": "receive.wav",
            "建图": "建图.wav",
            
            # 控制模式
            "遥控模式": "遥控模式.wav",
            "导航模式": "导航模式.wav",

            # 手动设置目标点位
            "标记为一号点位": "标记为一号点位.wav",
            "标记为二号点位": "标记为二号点位.wav",
            "标记为三号点位": "标记为三号点位.wav",
            "标记为四号点位": "标记为四号点位.wav",
            "标记为五号点位": "标记为五号点位.wav",
            "标记为六号点位": "标记为六号点位.wav",
            "标记为七号点位": "标记为七号点位.wav",
            "标记为八号点位": "标记为八号点位.wav",
            "标记为九号点位": "标记为九号点位.wav",

            # 其他命令
            "你好小益": "hello.wav",
            "蹲下": "down.wav",
            "起立": "stand_up.wav",
            "回家": "go_home.wav",
            "继续": "go.wav",
            "初始化成功": "init.wav",
            "避障":"work.wav",

        }
        
        # 默认回复音频文件
        default_audio = "go.wav"
        
        # 获取对应的音频文件
        audio_file = keyword_to_audio.get(keyword, default_audio)
        full_path = os.path.join(audio_dir, audio_file)
        
        # 检查文件是否存在
        if not os.path.isfile(full_path):
            rospy.logwarn(f"音频文件不存在: {full_path}")
            return
        
        # 确保输出设备已正确设置
        self.set_default_output_device()
        
        # 播放音频文件
        try:
            rospy.loginfo(f"播放音频: {audio_file}")
            
            # 动态查找USB2.0 Device的card ID
            usb_device_id = self.find_usb2_device_id()
            if usb_device_id is not None:
                # 设置音量为最大 (100%)
                try:
                    # 使用动态获取的声卡ID设置PCM音量为100%并解除静音
                    subprocess.run(['amixer', '-c', usb_device_id, 'set', 'PCM', '100%', 'unmute'], 
                                stdout=subprocess.DEVNULL, 
                                stderr=subprocess.DEVNULL)
                    rospy.loginfo(f"已将声卡{usb_device_id}的PCM音量设置为最大")
                except Exception as e:
                    rospy.logwarn(f"设置音量失败: {e}")
                # 使用找到的设备ID播放
                cmd = ['aplay', '-D', f'plughw:{usb_device_id},0', full_path]
                subprocess.Popen(cmd, 
                               stdout=subprocess.DEVNULL, 
                               stderr=subprocess.DEVNULL)
                # 记录使用的设备
                rospy.loginfo(f"使用设备播放声音: plughw:{usb_device_id},0 (USB2.0 Device)")
            else:
                # 如果没找到设备，使用默认命令
                rospy.logwarn("未找到USB2.0 Device，使用默认设备播放")
                cmd = ['aplay', full_path]
                subprocess.Popen(cmd, 
                               stdout=subprocess.DEVNULL, 
                               stderr=subprocess.DEVNULL)
        except Exception as e:
            rospy.logerr(f"播放音频时出错: {e}")


    def run(self):
        """Start capturing audio and performing keyword spotting."""
        try:
            # 设置音频输出设备
            self.set_default_output_device()
            
            # 设置音频输入流
            if self.setup_audio_stream():
                # 如果成功设置了音频流，就保持节点运行
                rospy.loginfo("关键词检测系统已启动，定时器将每2秒检查音频设备")
                rospy.spin()
            else:
                rospy.logerr("无法设置音频流，关闭节点")
                rospy.signal_shutdown("无法设置音频流")
        except Exception as e:
            rospy.logerr(f"运行关键词检测系统时出错: {e}")
            rospy.signal_shutdown(f"系统错误: {e}")




    def __del__(self):
        """析构函数，清理资源"""
        if hasattr(self, 'device_timer'):
            self.device_timer.shutdown()
        
        if hasattr(self, 'command_timer') and self.command_timer:
            self.command_timer.shutdown()
        
        if hasattr(self, 'audio_stream') and self.audio_stream:
            try:
                self.audio_stream.stop()
                self.audio_stream.close()
                rospy.loginfo("已关闭音频流")
            except Exception as e:
                rospy.logwarn(f"关闭音频流时出错: {e}")


    def find_usb2_device_id(self):
        """查找USB2.0 Device的card ID"""
        try:
            # 运行arecord -l命令获取声卡信息
            cmd = ["arecord", "-l"]
            result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            
            if result.returncode != 0:
                rospy.logwarn(f"执行arecord -l命令失败: {result.stderr}")
                return None
                
            # 在输出中查找USB2.0 Device设备
            lines = result.stdout.strip().split('\n')
            for line in lines:
                if "USB2.0 Device" in line:
                    # 尝试从行中提取card ID
                    # 行格式类似于 "card 0: Device_1 [USB2.0 Device], device 0: USB Audio [USB Audio]"
                    match = re.search(r"card\s+(\d+)", line)
                    if match:
                        card_id = match.group(1)
                        rospy.loginfo(f"找到USB2.0 Device，card ID: {card_id}")
                        return card_id
            
            rospy.logwarn("未在arecord输出中找到USB2.0 Device")
            return None
            
        except Exception as e:
            rospy.logerr(f"查找USB2.0 Device时出错: {e}")
            return None
        

if __name__ == "__main__":
    try:
        node = KeywordSpotterNode()
        rospy.loginfo("启动关键词检测...")
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down Keyword Spotter Node.")