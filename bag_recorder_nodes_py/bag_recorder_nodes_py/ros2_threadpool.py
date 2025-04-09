from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import time
import random
import logging
# import threading
import asyncio
# from concurrent.futures import Future



class ROS2ThreadPool(Node):
    def __init__(self, node_name: str = 'ros2_thread_pool', num_threads: int = None):
        """
        ROS2多线程任务处理器
        
        :param node_name: 节点名称
        :param num_threads: 线程数(默认使用CPU核心数)
        """
        super().__init__(node_name)
        self._executor = MultiThreadedExecutor(num_threads=num_threads)
        self._executor.add_node(self)
        self._running = False
        self._logger = self.get_logger()
        self._task_id = 0
        
    def submit_task(self, task_func: callable, *args, **kwargs) -> bool:
        """
        提交任务到ROS2执行器
        
        :param task_func: 要执行的任务函数
        :param args: 函数位置参数
        :param kwargs: 函数关键字参数
        :return: 是否成功提交
        """
        if not self._running:
            self._logger.warn("执行器未运行，任务未被提交")
            return False
            
        try:
            self._task_id += 1
            task_name = f"task_{self._task_id}"
            
            # 包装任务函数以添加日志
            def wrapped_task():
                try:
                    # self._logger.info(f"开始执行任务: {task_name}")
                    start_time = time.time()
                    result = task_func(*args, **kwargs)
                    elapsed = time.time() - start_time
                    # self._logger.info(f"完成任务: {task_name}, 耗时: {elapsed:.2f}s")
                    return result
                except Exception as e:
                    self._logger.error(f"fail mission: {task_name}, fault: {str(e)}")
                    raise
            
            # 创建并提交任务
            self._executor.create_task(wrapped_task)
            return True
            
        except Exception as e:
            self._logger.error(f"submit task fail: {str(e)}")
            return False
    
    def start(self):
        """启动执行器"""
        if not self._running:
            self._running = True
            self._logger.info("ROS2 Thread Pool start.")
            
            # 在后台线程中运行执行器
            import threading
            self._executor_thread = threading.Thread(
                target=self._executor.spin, 
                daemon=True
            )
            self._executor_thread.start()
    
    def stop(self):
        """停止执行器"""
        if self._running:
            self._running = False
            self._executor.shutdown()
            self._logger.info("ROS2 Thread Pool stop.")
    
    def __enter__(self):
        self.start()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()
        self.destroy_node()
    
    @property
    def is_running(self) -> bool:
        """执行器是否正在运行"""
        return self._running
