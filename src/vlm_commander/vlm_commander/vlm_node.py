#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import json
import torch
from transformers import LlavaNextForConditionalGeneration, LlavaNextProcessor
from transformers import BitsAndBytesConfig
from PIL import Image as PILImage
import numpy as np

class VLMCommander(Node):
    def __init__(self):
        super().__init__('vlm_commander')
        
        self.get_logger().info('🤖 VLM Commander starting...')
        
        # Use mock VLM for now
        self.declare_parameter('use_real_vlm', False)
        self.use_real_vlm = self.get_parameter('use_real_vlm').value
        # Load LLaVA if requested
        self.vlm_model = None
        self.vlm_processor = None
    
        if self.use_real_vlm:
            self.get_logger().info('🔧 Loading LLaVA 1.6 model...')
            self.load_llava()
        
        self.bridge = CvBridge()
        self.current_image = None
        
        # Subscribe to camera
        self.img_sub = self.create_subscription(
            Image,
            '/camera',
            self.image_callback,
            10
        )
        
        # Subscribe to user commands
        self.cmd_sub = self.create_subscription(
            String,
            '/user_command',
            self.command_callback,
            10
        )

        # Publisher → cube_pusher_node filters its work queue from this
        self.targets_pub = self.create_publisher(String, '/target_cubes', 10)

        self.get_logger().info('✅ VLM Commander ready!')
        self.get_logger().info('📷 Subscribed to: /camera')
        self.get_logger().info('📤 Publishing targets to: /target_cubes')
    def load_llava(self):
        """Load LLaVA 1.6 model with 4-bit quantization"""
        try:
            quant_config = BitsAndBytesConfig(
                load_in_4bit=True,
                bnb_4bit_compute_dtype=torch.float16,
                bnb_4bit_use_double_quant=True,
                bnb_4bit_quant_type="nf4"
            )
        
            model_id = "llava-hf/llava-v1.6-mistral-7b-hf"
        
            self.vlm_model = LlavaNextForConditionalGeneration.from_pretrained(
                model_id,
                quantization_config=quant_config,
                device_map="auto",
                low_cpu_mem_usage=True,
                torch_dtype=torch.float16
            )
        
            self.vlm_processor = LlavaNextProcessor.from_pretrained(model_id)
        
            vram_gb = torch.cuda.memory_allocated() / 1e9
            self.get_logger().info(f'✅ LLaVA loaded! VRAM: {vram_gb:.2f} GB')
        
        except Exception as e:
            self.get_logger().error(f'❌ Failed to load LLaVA: {e}')
            self.use_real_vlm = False
        
    def image_callback(self, msg):
        """Store latest camera image"""
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'❌ Image callback error: {e}')
    
    def command_callback(self, msg):
        """Process user command"""
        command = msg.data
        self.get_logger().info(f'📝 Received command: "{command}"')
        
        if self.current_image is None:
            self.get_logger().warn('⚠️  No camera image available yet')
            return
        
        # Query VLM
        start_time = self.get_clock().now()
        
        if self.use_real_vlm:
            response = self.query_real_vlm(self.current_image, command)
        else:
            response = self.query_mock_vlm(command)
        
        elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
        self.get_logger().info(f'⏱️  VLM processing: {elapsed:.3f}s')
        self.get_logger().info(f'🧠 VLM response: {response}')
        
        # Execute plan
        self.execute_plan(response)
    
    def query_mock_vlm(self, command):
        """Mock VLM - smarter keyword matching based on actual cube positions"""
        import json
    
        cmd_lower = command.lower()
    
        # Map of cube colors to their actual positions
        cube_info = {
            'red': {'pos': (1.0, 0.5), 'desc': 'right side, slightly forward'},
            'blue': {'pos': (-1.0, -0.5), 'desc': 'left side, slightly back'},
            'green': {'pos': (0.0, -1.0), 'desc': 'center-back area'},
            'yellow': {'pos': (0.0, 1.0), 'desc': 'center-forward area'}
        }
    
        # Detect which cube(s) are requested
        target_cubes = []
        for color, info in cube_info.items():
            if color in cmd_lower:
                target_cubes.append({
                    "color": color,
                    "position_description": f"{color} cube at {info['desc']}",
                    "coordinates": {"x": info['pos'][0], "y": info['pos'][1]},
                    "priority": 1
                })
    
        # If no specific color, check for semantic locations
        if not target_cubes:
            if 'right' in cmd_lower:
                target_cubes.append({
                    "color": "red",
                    "position_description": "red cube on right side",
                    "coordinates": {"x": 1.0, "y": 0.5},
                    "priority": 1
                })
            elif 'left' in cmd_lower:
                target_cubes.append({
                    "color": "blue",
                    "position_description": "blue cube on left side",
                    "coordinates": {"x": -1.0, "y": -0.5},
                    "priority": 1
                })
            elif 'center' in cmd_lower or 'middle' in cmd_lower:
                target_cubes.extend([
                    {
                        "color": "green",
                        "position_description": "green cube at center-back",
                        "coordinates": {"x": 0.0, "y": -1.0},
                        "priority": 1
                    },
                    {
                        "color": "yellow",
                        "position_description": "yellow cube at center-front",
                        "coordinates": {"x": 0.0, "y": 1.0},
                        "priority": 2
                    }
                ])
            else:
                # Default: all cubes
                target_cubes = [
                    {"color": "red", "position_description": "red", "coordinates": {"x": 1.0, "y": 0.5}, "priority": 1},
                    {"color": "blue", "position_description": "blue", "coordinates": {"x": -1.0, "y": -0.5}, "priority": 2},
                    {"color": "green", "position_description": "green", "coordinates": {"x": 0.0, "y": -1.0}, "priority": 3},
                    {"color": "yellow", "position_description": "yellow", "coordinates": {"x": 0.0, "y": 1.0}, "priority": 4}
                ]
    
        return json.dumps({
            "reasoning": f"User requested {len(target_cubes)} cube(s): {[c['color'] for c in target_cubes]}",
            "target_cubes": target_cubes,
            "obstacles": ["desks", "chairs", "teacher desk at (3.5, 0)"],
            "collection_zone": {"x": -4.0, "y": -4.0},
            "suggested_order": "Navigate to each cube, cube_collector handles fine positioning and pushing"
        })
    
    def query_real_vlm(self, cv_image, command):
        """Real LLaVA inference"""
        try:
            # Convert OpenCV image to PIL
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            pil_image = PILImage.fromarray(rgb_image)
        
            # Craft prompt for robot task understanding
            prompt = f"""[INST] <image>
    You are controlling a classroom cleaning robot. The robot needs to collect colored cubes.

    Current command: "{command}"

    Analyze the image and provide a JSON response with:
    {{
      "reasoning": "brief description of what you see",
      "target_cubes": [
        {{
          "color": "red/blue/green/yellow",
          "position_description": "descriptive location in the scene",
          "priority": 1
        }}
      ],
      "obstacles": ["list any obstacles you see"],
      "suggested_approach": "how to reach the cubes"
    }}

    Respond ONLY with valid JSON, no other text.
    [/INST]"""
        
            # Process inputs
            inputs = self.vlm_processor(text=prompt, images=pil_image, return_tensors="pt")
            inputs = {k: v.to(self.vlm_model.device) for k, v in inputs.items()}
        
            # Generate response
            with torch.inference_mode():
                output = self.vlm_model.generate(
                    **inputs,
                    max_new_tokens=300,
                    temperature=0.2,
                    do_sample=True
                )
        
            # Decode response
            response = self.vlm_processor.decode(output[0], skip_special_tokens=True)
        
            # Extract JSON from response (after [/INST])
            response = response.split("[/INST]")[-1].strip()
        
            # Clean up JSON if needed
            if "```json" in response:
                response = response.split("```json")[1].split("```")[0].strip()
            elif "```" in response:
                response = response.split("```")[1].split("```")[0].strip()
        
            return response
        
        except Exception as e:
            self.get_logger().error(f'❌ LLaVA inference error: {e}')
            # Fallback to mock
            return self.query_mock_vlm(command)
    
    def execute_plan(self, vlm_response):
        """Publish identified cube colours to /target_cubes.
        cube_pusher_node receives this and restricts its work queue accordingly.
        """
        try:
            plan   = json.loads(vlm_response)
            self.get_logger().info(f'💡 Strategy: {plan.get("reasoning", "N/A")}')

            cubes  = plan.get('target_cubes', [])
            colors = [c['color'] for c in cubes if 'color' in c]

            if not colors:
                self.get_logger().warn('⚠️  No target cubes identified by VLM')
                return

            msg      = String()
            msg.data = json.dumps({'target_colors': colors})
            self.targets_pub.publish(msg)
            self.get_logger().info(f'📤 Sent to cube_pusher → {colors}')

        except json.JSONDecodeError as e:
            self.get_logger().error(f'❌ JSON parse error: {e}')
        except Exception as e:
            self.get_logger().error(f'❌ Execution error: {e}')
    


def main(args=None):
    rclpy.init(args=args)
    node = VLMCommander()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
