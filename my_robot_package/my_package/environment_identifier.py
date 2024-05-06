import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from rclpy.time import Time
import matplotlib.pyplot as plt
from skimage.metrics import structural_similarity as ssim


class EnvironmentIdentifier(Node):
    def __init__(self):
        super().__init__('environment_identifier')
        self.publisher = self.create_publisher(String, '/environment', 10)
        self.timer = self.create_timer(10, self.identify_environment)
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            1)
        self.input_grid = None

        
        self.predefined_maps = {
            'A': np.random.rand(50, 50),
            'B': np.random.rand(50, 50),
            'C': np.random.rand(50, 50),
            'D': np.random.rand(50, 50),
            'E': np.random.rand(50, 50)
        }

       
        self.obstacles = {
            'A': [("circle", 3.75, (35, 35)), ("circle", 3.75, (15, 35)), ("circle", 3.75, (35, 15))],
            'B': [("circle", 3.75, (35, 35)), ("circle", 3.75, (35, 15)), ("circle", 3.75, (15, 15)), ("circle", 3.75, (15, 35))],
            'C': [("circle", 5, (35, 25)), ("circle", 6.25, (25, 25)), ("circle", 7.5, (15, 25))],
            'D': [("circle", 5, (35, 35)), ("circle", 5, (35, 15)), ("circle", 5, (35, 25)), ("circle", 5, (25, 35))], #("circle", 5, (15, 35))],
            'E': [("circle", 3.75, (35, 35)), ("circle", 3.75, (35, 15)), ("circle", 3.75, (15, 15)), ("circle", 3.75, (15, 35)), ("circle", 5, (25, 25))]
        }

        self.best_score = float('-inf')
        self.best_environment = None
        self.last_map_time = self.get_clock().now()
        self.initialize_obstacles_in_maps()
        #self.plot_all_maps()
        self.no_map_timer = self.create_timer(2, self.check_no_map_received)

    def fill_circle(self, grid, center, radius, value=35):
        cx, cy = center
        for x in range(max(0, cx - radius), min(grid.shape[0], cx + radius + 1)):
            for y in range(max(0, cy - radius), min(grid.shape[1], cy + radius + 1)):
                if (x - cx)**2 + (y - cy)**2 <= radius**2 and (x - cx)**2 + (y - cy)**2 >= (radius - 1)**2:
                    grid[x, y] = value


    def initialize_obstacles_in_maps(self):
        for env, obs_list in self.obstacles.items():
            for shape, radius, center in obs_list:
                if shape == "circle":
                    self.fill_circle(self.predefined_maps[env], center, int(radius))


    def plot_all_maps(self):
        for env, grid_map in self.predefined_maps.items():
            self.plot_grid_map(grid_map, env)

    def plot_grid_map(self, grid_map, environment_name):
        plt.figure(figsize=(10, 10))
        plt.title(f"Map Representation for Environment {environment_name}")
        plt.imshow(grid_map, cmap='hot', interpolation='nearest', origin='lower')
        plt.colorbar()
        plt.show()
    
    def plot_overlay_map(self, input_grid, best_match_grid, input_label='Input Environment', best_match_label='Best Match Environment'):
        plt.figure(figsize=(12, 6))

        input_norm = (input_grid - np.min(input_grid)) / (np.max(input_grid) - np.min(input_grid))
        best_match_norm = (best_match_grid - np.min(best_match_grid)) / (np.max(best_match_grid) - np.min(best_match_grid))


        plt.subplot(1, 2, 1)
        plt.title(input_label)
        plt.imshow(input_norm, cmap='viridis', interpolation='nearest', alpha=0.75)
        plt.colorbar()
        plt.grid(False)


        plt.subplot(1, 2, 2)
        plt.title(best_match_label)
        plt.imshow(best_match_norm, cmap='viridis', interpolation='nearest', alpha=0.75)
        plt.colorbar()
        plt.grid(False)


        plt.imshow(best_match_norm, cmap='hot', alpha=0.5, interpolation='nearest')
        plt.colorbar()
        plt.grid(False)
        plt.show()

    def map_callback(self, msg):
        width, height = msg.info.width, msg.info.height
        expected_size = width * height
        actual_size = len(msg.data)

        if actual_size != expected_size:
            print(f"Data size mismatch. Expected {expected_size}, but got {actual_size}.")
            return  

       

        data = np.array(msg.data).reshape((height, width))
        self.input_grid = data
        self.last_map_time = self.get_clock().now()

        self.publish_identified_environment(self.best_environment)
        

    def check_no_map_received(self):
        if (self.get_clock().now() - self.last_map_time).nanoseconds / 1e9 >= 2:
            if self.best_environment is not None and self.best_environment in self.predefined_maps:
                
                best_match_map = self.predefined_maps[self.best_environment]

               
                self.plot_overlay_map(self.input_grid, best_match_map, 'Input Environment', self.best_environment)

               
                self.publish_identified_environment(self.best_environment)
                self.best_score = float('-inf')
                self.best_environment = None
                print("No new map received for 2 seconds. Published best environment.")
            else:
                print("No best environment to publish or plot.")


    def identify_environment(self):
        if self.input_grid is not None:
            identified_environment, score = self.compare_maps(self.input_grid, self.predefined_maps)
            if score > self.best_score:
                self.best_score = score
                self.best_environment = identified_environment

    def normalize_grid(self, grid):
        grid_min = np.min(grid)
        grid_max = np.max(grid)
        if grid_max - grid_min != 0:
            return (grid - grid_min) / (grid_max - grid_min)
        else:
            return grid


    def compare_maps(self, input_grid, predefined_maps):
        scores = {}
        distances = {}
        correlations = {}
        ssims = {}
        
        normalized_input = self.normalize_grid(input_grid)
        for map_name, map_grid in predefined_maps.items():
            if input_grid.shape == map_grid.shape:  
                
                normalized_map = self.normalize_grid(map_grid)
                
                distance = np.linalg.norm(normalized_input.flatten() - normalized_map.flatten())

                correlation = np.corrcoef(normalized_input.flatten(), normalized_map.flatten())[0, 1]

                ssim_index = ssim(normalized_input, normalized_map, data_range=normalized_map.max() - normalized_map.min())

                #combined_score = (1 - distance) * 0.5 + correlation * 0.25 + ssim_index * 0.25
                combined_score = (-distance) * 0.1 + ssim_index * 0.2 + correlation * 12
                if distance > 11.0 and (correlation + ssim_index) < 0.5:
                    combined_score -= 10.0
                scores[map_name] = combined_score
                distances[map_name] = distance
                correlations[map_name] = correlation

                print("--------------------------------------")
                print(f"Distance score for {map_name}: {distance}")
                print(f"Correlation score for {map_name}: {correlation}")
                print(f"SSIM index score for {map_name}: {ssim_index}")
                print(f"Combined score for {map_name}: {combined_score}")  

        if scores:
            
            best_match = max(scores, key=scores.get)
            best_score = scores[best_match]
            print(f"Best match: {best_match} with score: {best_score}")  
            return best_match, best_score
        else:
            return "No valid comparisons", float('-inf')  


    def publish_identified_environment(self, environment):
        obstacle_info = self.get_obstacle_description(environment)
        msg = String()
        msg.data = f"The identified environment is: {environment}\n{obstacle_info}"
        self.publisher.publish(msg)
        self.get_logger().info(msg.data)

    def get_obstacle_description(self, environment):
        if environment in self.obstacles:
            description = f"Environment {environment}\t"
            for idx, (shape, size, (x, y)) in enumerate(self.obstacles[environment]):
                description += f"Obstacle {idx + 1}: A {shape} with radius {size} centered at ({x}, {y})\n"
            return description.strip()
        else:
            return f"No obstacles defined for environment {environment}"

def main(args=None):
    rclpy.init(args=args)
    environment_identifier = EnvironmentIdentifier()
    rclpy.spin(environment_identifier)
    environment_identifier.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
