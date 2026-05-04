"""
RRT (Rapidly-Exploring Random Trees) Motion Planning Simulation
Student NIM: 4222301027
Course: RE605 - Motion Planning
Environment: Figure 1 - Obstacles disesuaikan dengan gambar yang diberikan
Start: (0, 30), Goal: (50, 50)
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
import random

class RRTPlanner:
    """
    Rapidly-Exploring Random Trees (RRT) path planning algorithm
    """
    
    def __init__(self, start, goal, obstacle_list, map_bounds, 
                 step_size=2.0, max_iter=5000, goal_sample_rate=0.1):
        """
        Initialize RRT planner
        
        Args:
            start: Start position (x, y)
            goal: Goal position (x, y)
            obstacle_list: List of obstacles [(x_min, y_min, x_max, y_max), ...]
            map_bounds: Map boundaries (x_min, y_min, x_max, y_max)
            step_size: Maximum extension distance
            max_iter: Maximum iterations
            goal_sample_rate: Probability of sampling goal
        """
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        self.obstacle_list = obstacle_list
        self.map_bounds = map_bounds
        self.step_size = step_size
        self.max_iter = max_iter
        self.goal_sample_rate = goal_sample_rate
        self.node_list = [self.start]
        self.path = []
        
    def plan(self):
        """
        Execute RRT planning algorithm
        
        Returns:
            path: List of nodes forming the path from start to goal
        """
        print("Starting RRT planning...")
        print(f"Start: ({self.start.x}, {self.start.y})")
        print(f"Goal: ({self.goal.x}, {self.goal.y})")
        
        for i in range(self.max_iter):
            # Sample random point (with bias toward goal)
            if random.random() < self.goal_sample_rate:
                random_node = Node(self.goal.x, self.goal.y)
            else:
                random_node = self.get_random_node()
            
            # Find nearest node in tree
            nearest_node = self.get_nearest_node(random_node)
            
            # Extend tree toward random node
            new_node = self.extend(nearest_node, random_node)
            
            # Check if path to new node is collision-free
            if new_node and not self.check_collision(nearest_node, new_node):
                self.node_list.append(new_node)
                
                # Check if goal is reached
                if self.calculate_distance(new_node, self.goal) <= self.step_size:
                    if not self.check_collision(new_node, self.goal):
                        final_node = Node(self.goal.x, self.goal.y)
                        final_node.parent = new_node
                        self.node_list.append(final_node)
                        self.path = self.generate_path()
                        print(f"Goal reached in {i+1} iterations!")
                        print(f"Path length: {len(self.path)} nodes")
                        print(f"Path distance: {self.calculate_path_distance():.2f} units")
                        return self.path
            
            if (i + 1) % 500 == 0:
                print(f"Iteration {i+1}/{self.max_iter}, Nodes: {len(self.node_list)}")
        
        print("Failed to find path within maximum iterations")
        return None
    
    def get_random_node(self):
        """Generate random node within map bounds"""
        x = random.uniform(self.map_bounds[0], self.map_bounds[2])
        y = random.uniform(self.map_bounds[1], self.map_bounds[3])
        return Node(x, y)
    
    def get_nearest_node(self, random_node):
        """Find nearest node in tree to random node"""
        distances = [self.calculate_distance(node, random_node) 
                    for node in self.node_list]
        min_index = distances.index(min(distances))
        return self.node_list[min_index]
    
    def extend(self, from_node, to_node):
        """
        Extend tree from from_node toward to_node
        """
        distance = self.calculate_distance(from_node, to_node)
        
        if distance <= self.step_size:
            new_x = to_node.x
            new_y = to_node.y
        else:
            # Extend by step_size in direction of to_node
            theta = np.arctan2(to_node.y - from_node.y, to_node.x - from_node.x)
            new_x = from_node.x + self.step_size * np.cos(theta)
            new_y = from_node.y + self.step_size * np.sin(theta)
        
        new_node = Node(new_x, new_y)
        new_node.parent = from_node
        return new_node
    
    def check_collision(self, from_node, to_node, num_checks=20):
        """
        Check if path between two nodes collides with obstacles
        """
        for i in range(num_checks + 1):
            t = i / num_checks
            x = from_node.x + t * (to_node.x - from_node.x)
            y = from_node.y + t * (to_node.y - from_node.y)
            
            # Check against all obstacles
            for obs in self.obstacle_list:
                if obs[0] <= x <= obs[2] and obs[1] <= y <= obs[3]:
                    return True  # Collision detected
        
        return False  # No collision
    
    def calculate_distance(self, node1, node2):
        """Calculate Euclidean distance between two nodes"""
        return np.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)
    
    def generate_path(self):
        """Generate path from start to goal by backtracking through parents"""
        path = []
        node = self.node_list[-1]  # Goal node
        
        while node is not None:
            path.append(node)
            node = node.parent
        
        path.reverse()
        return path
    
    def calculate_path_distance(self):
        """Calculate total path distance"""
        if not self.path:
            return 0
        
        total_distance = 0
        for i in range(len(self.path) - 1):
            total_distance += self.calculate_distance(self.path[i], self.path[i+1])
        
        return total_distance


class Node:
    """
    Node class for RRT tree
    """
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None


def create_figure1_obstacles_exact():
    """
    Create obstacle list PERSIS sesuai penjelasan user
    
    Boundary: Box dari (-10,-10) ke (60,60)
    Obstacle 1: y40 x-10->x10, y20 x10->x40, + branch y20 x30->y5 x30->y5 x10
    Obstacle 2: y60 x30->y40 x30->y40 x40
    Obstacle 3: y5 x60->y5 x40
    """
    obstacles = []
    
    # ============================================
    # BOUNDARY (Kotak penuh) - gunakan ketebalan kecil agar garis pas di koordinat
    # ============================================
    thickness = 0.5
    obstacles.append((-10, -10, -10 + thickness, 60))     # Left wall (x=-10)
    obstacles.append((60, -10, 60 + thickness, 60))       # Right wall (x=60)
    obstacles.append((-10, -10, 62, -10 + thickness))     # Bottom wall (y=-10)
    obstacles.append((-10, 60, 62, 60 + thickness))       # Top wall (y=60)
    
    # ============================================
    # OBSTACLE 1: Pola sesuai penjelasan pengguna (kontinyu, tanpa celah)
    # Semua segment dibuat tepat pada koordinat x/y dan menggunakan `thickness` kecil
    # Horizontal atas: y=40, x=-10..10
    obstacles.append((-10, 40, 11, 40 + thickness))

    # Vertical connector at x=10: y=40 down to y=20
    obstacles.append((10, 20, 10 + thickness, 40))

    # Horizontal middle: y=20, x=10..40
    obstacles.append((10, 20, 40, 20 + thickness))

    # Branch vertical from x=30, y=20 down to y=5
    obstacles.append((30, 5, 30 + thickness, 20))

    # Branch horizontal bottom: y=5, from x=10 to x=30
    obstacles.append((10, 5, 30, 5 + thickness))
    
    # ============================================
    # OBSTACLE 2: Vertikal dan horizontal
    # ============================================
    # Vertikal wall: x=30, y=40 to y=60 (use thickness)
    obstacles.append((30, 40, 30 + thickness, 60))

    # Horizontal wall: y=40, x=30 to x=40 (use thickness)
    obstacles.append((30, 40, 40, 40 + thickness))
    
    # ============================================
    # OBSTACLE 3: Horizontal wall bawah kanan
    # ============================================
    # Horizontal wall: y=5, x=40 to x=60 (use thickness)
    obstacles.append((40, 5, 60, 5 + thickness))
    
    return obstacles


def visualize_rrt(rrt_planner, save_path='rrt_simulation_exact.png'):
    """
    Visualize RRT tree and path - PERSIS seperti gambar asli
    """
    fig, ax = plt.subplots(1, 1, figsize=(14, 10))
    
    # Draw obstacles dengan warna hitam solid
    for obs in rrt_planner.obstacle_list:
        width = obs[2] - obs[0]
        height = obs[3] - obs[1]
        rect = patches.Rectangle((obs[0], obs[1]), width, height, 
                                 linewidth=0, edgecolor='none', 
                                 facecolor='black', alpha=1.0)
        ax.add_patch(rect)
    
    # Draw all tree edges (cyan, semi-transparent)
    for node in rrt_planner.node_list:
        if node.parent:
            ax.plot([node.x, node.parent.x], [node.y, node.parent.y], 
                   'c-', linewidth=0.5, alpha=0.3, label='Tree' if node == rrt_planner.node_list[1] else '')
    
    # Draw tree nodes (cyan dots, kecil)
    node_x = [node.x for node in rrt_planner.node_list]
    node_y = [node.y for node in rrt_planner.node_list]
    ax.plot(node_x, node_y, 'c.', markersize=2, alpha=0.4)
    
    # Draw path (merah tebal dengan marker kuning)
    if rrt_planner.path:
        path_x = [node.x for node in rrt_planner.path]
        path_y = [node.y for node in rrt_planner.path]
        ax.plot(path_x, path_y, 'r-', linewidth=3, label='Planned Path', zorder=10)
        ax.plot(path_x, path_y, 'yo', markersize=5, zorder=11)
    
    # Draw start (green circle) - PERSIS seperti gambar
    ax.plot(rrt_planner.start.x, rrt_planner.start.y, 'o', 
           color='green', markersize=12, label='Start (0,30)', zorder=15)
    
    # Draw goal (blue X) - PERSIS seperti gambar
    ax.plot(rrt_planner.goal.x, rrt_planner.goal.y, 'x', 
           color='blue', markersize=15, markeredgewidth=3, label='Goal (50,50)', zorder=15)
    
    # Set axis properties
    ax.set_xlim(-25, 65)
    ax.set_ylim(-15, 65)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3, linewidth=0.5)
    ax.set_xlabel('X Position', fontsize=12)
    ax.set_ylabel('Y Position', fontsize=12)
    ax.set_title('RRT Algorithm - Motion Planning\nStart (0,30) → Goal (50,50)', 
                fontsize=14, fontweight='bold')
    ax.legend(loc='upper left', fontsize=10)
    
    # Set background putih
    ax.set_facecolor('white')
    fig.patch.set_facecolor('white')
    
    plt.tight_layout()
    plt.savefig(save_path, dpi=300, bbox_inches='tight', facecolor='white')
    print(f"\nVisualization saved to: {save_path}")
    plt.close()
    
    return fig


def print_path_details(rrt_planner):
    """
    Print detailed path information
    """
    if not rrt_planner.path:
        print("\nNo path found!")
        return
    
    print("\n" + "="*70)
    print("DETAIL JALUR YANG DITEMUKAN")
    print("="*70)
    print(f"Jumlah waypoints: {len(rrt_planner.path)}")
    print(f"Total jarak jalur: {rrt_planner.calculate_path_distance():.2f} units")
    print(f"Jumlah node yang dieksplorasi: {len(rrt_planner.node_list)}")
    
    # Hitung jarak garis lurus
    straight_distance = rrt_planner.calculate_distance(rrt_planner.start, rrt_planner.goal)
    efficiency = (straight_distance / rrt_planner.calculate_path_distance()) * 100
    print(f"Jarak garis lurus: {straight_distance:.2f} units")
    print(f"Efisiensi jalur: {efficiency:.1f}%")
    
    print("\nWaypoints jalur (10 pertama dan 10 terakhir):")
    print("-" * 70)
    print(f"{'No':<6} {'X':<12} {'Y':<12} {'Jarak Segmen':<20}")
    print("-" * 70)
    
    # Print first 10 waypoints
    for i in range(min(10, len(rrt_planner.path))):
        node = rrt_planner.path[i]
        if i > 0:
            dist = rrt_planner.calculate_distance(rrt_planner.path[i-1], node)
            print(f"{i:<6} {node.x:<12.2f} {node.y:<12.2f} {dist:<20.2f}")
        else:
            print(f"{i:<6} {node.x:<12.2f} {node.y:<12.2f} {'START':<20}")
    
    # Print ellipsis if path is long
    if len(rrt_planner.path) > 20:
        print(f"{'...':<6} {'...':<12} {'...':<12} {'...':<20}")
    
    # Print last 10 waypoints
    if len(rrt_planner.path) > 10:
        start_idx = max(10, len(rrt_planner.path) - 10)
        for i in range(start_idx, len(rrt_planner.path)):
            node = rrt_planner.path[i]
            dist = rrt_planner.calculate_distance(rrt_planner.path[i-1], node)
            if i == len(rrt_planner.path) - 1:
                print(f"{i:<6} {node.x:<12.2f} {node.y:<12.2f} {dist:<20.2f} ← GOAL")
            else:
                print(f"{i:<6} {node.x:<12.2f} {node.y:<12.2f} {dist:<20.2f}")
    
    print("="*70)


def print_obstacle_info(obstacles):
    """
    Print informasi obstacle untuk verifikasi
    """
    print("\n" + "="*70)
    print("INFORMASI OBSTACLE")
    print("="*70)
    print(f"Total obstacle: {len(obstacles)}")
    print("\nDetail setiap obstacle (x_min, y_min, x_max, y_max):")
    print("-" * 70)
    
    for i, obs in enumerate(obstacles, 1):
        width = obs[2] - obs[0]
        height = obs[3] - obs[1]
        print(f"Obstacle {i:2d}: ({obs[0]:6.1f}, {obs[1]:6.1f}, {obs[2]:6.1f}, {obs[3]:6.1f}) "
              f"- Size: {width:.1f} x {height:.1f}")
    print("="*70)


def main():
    """
    Main function to run RRT motion planning simulation
    """
    print("="*70)
    print("SIMULASI RRT MOTION PLANNING")
    print("RE605 - Motion Planning")
    print("Student NIM: 4222301027")
    print("="*70)
    print("\nOBSTACLE DISESUAIKAN DENGAN GAMBAR YANG DIBERIKAN")
    
    # Set random seed for reproducibility
    random.seed(42)
    np.random.seed(42)
    
    # Define start and goal positions (SESUAI GAMBAR)
    start = (0, 30)   # Green circle
    goal = (50, 50)   # Blue X
    
    # Create obstacles PERSIS dari gambar
    obstacles = create_figure1_obstacles_exact()
    
    # Print obstacle information
    print_obstacle_info(obstacles)
    
    # Define map bounds
    map_bounds = (-20, -10, 62, 62)
    
    # Initialize RRT planner
    print("\nMenginisialisasi RRT Planner...")
    rrt = RRTPlanner(
        start=start,
        goal=goal,
        obstacle_list=obstacles,
        map_bounds=map_bounds,
        step_size=2.5,        # Extension step size
        max_iter=5000,        # Maximum iterations
        goal_sample_rate=0.15 # 15% chance to sample goal
    )
    
    # Run planning algorithm
    print("\nMenjalankan algoritma RRT...")
    path = rrt.plan()
    
    # Print path details
    print_path_details(rrt)
    
    # Visualize results
    if path:
        print("\nMembuat visualisasi...")
        visualize_rrt(rrt, save_path='rrt_simulation_exact.png')
        print("\n✓ Simulasi berhasil!")
        print("✓ Jalur dari Start (0,30) ke Goal (50,50) ditemukan!")
    else:
        print("\n✗ Gagal menemukan jalur. Coba tingkatkan max_iter atau sesuaikan step_size.")
    
    return rrt


if __name__ == "__main__":
    rrt_planner = main()