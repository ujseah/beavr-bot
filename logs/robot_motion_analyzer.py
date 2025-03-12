import json
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation

class RobotMotionAnalyzer:
    def __init__(self):
        self.translation_diffs = []
        self.rotation_diffs = []
        
    def update_stats(self, curr_pos, curr_rot, prev_pos, prev_rot):
        """Update motion statistics."""
        # Calculate translation difference
        trans_diff = np.linalg.norm(curr_pos - prev_pos)
        self.translation_diffs.append(trans_diff)
        
        # Calculate rotation difference (in degrees)
        rel_rot = np.dot(prev_rot.T, curr_rot)
        trace = np.clip((np.trace(rel_rot) - 1) / 2, -1.0, 1.0)
        rot_diff = np.degrees(np.arccos(trace))
        self.rotation_diffs.append(rot_diff)
        
        # Calculate statistics with quantiles
        quantiles = [0.1, 0.25, 0.5, 0.75, 0.9]
        trans_stats = {
            'max': np.max(self.translation_diffs),
            'min': np.min(self.translation_diffs),
            'avg': np.mean(self.translation_diffs),
            'quantiles': {q: np.quantile(self.translation_diffs, q) for q in quantiles}
        }
        
        rot_stats = {
            'max': np.max(self.rotation_diffs),
            'min': np.min(self.rotation_diffs),
            'avg': np.mean(self.rotation_diffs),
            'quantiles': {q: np.quantile(self.rotation_diffs, q) for q in quantiles}
        }
        
        return trans_stats, rot_stats
    
    def plot_final_stats(self, trans_stats, rot_stats):
        """Plot final motion statistics with quantiles."""
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))
            
        # Plot translation stats
        x = ['Q10', 'Q25', 'Q50', 'Q75', 'Q90', 'Avg']
        y = [
            trans_stats['quantiles'][0.1],
            trans_stats['quantiles'][0.25],
            trans_stats['quantiles'][0.5],
            trans_stats['quantiles'][0.75],
            trans_stats['quantiles'][0.9],
            trans_stats['avg']
        ]
        colors = ['lightblue', 'blue', 'purple', 'blue', 'lightblue', 'yellow']
        ax1.bar(x, y, color=colors)
        ax1.set_title('Translation Statistics (meters)')
        ax1.set_ylabel('Distance (m)')
        
        # Plot rotation stats
        y = [
            rot_stats['quantiles'][0.1],
            rot_stats['quantiles'][0.25],
            rot_stats['quantiles'][0.5],
            rot_stats['quantiles'][0.75],
            rot_stats['quantiles'][0.9],
            rot_stats['avg']
        ]
        ax2.bar(x, y, color=colors)
        ax2.set_title('Rotation Statistics (degrees)')
        ax2.set_ylabel('Angle (degrees)')
        
        plt.tight_layout()
        plt.show()

def load_pose_log(log_file):
    with open(log_file, 'r') as f:
        return json.load(f)

def main():
    analyzer = RobotMotionAnalyzer()
    
    # Load pose log
    log_file = "logs/pose_log_20250122_141945.json"
    log_data = load_pose_log(log_file)
    
    prev_pos = None
    prev_rot = None
    
    # Skip first frame, analyze from second frame onwards
    for frame in log_data[1:]:
        # Extract current position and rotation from transformed_pose
        curr_pos = np.array([frame['transformed_pose'][i][3] for i in range(3)])
        curr_rot = np.array([[frame['transformed_pose'][i][j] for j in range(3)] for i in range(3)])
        
        if prev_pos is not None and prev_rot is not None:
            trans_stats, rot_stats = analyzer.update_stats(curr_pos, curr_rot, prev_pos, prev_rot)
        
        prev_pos = curr_pos.copy()
        prev_rot = curr_rot.copy()
    
    # Show final statistics
    analyzer.plot_final_stats(trans_stats, rot_stats)

if __name__ == "__main__":
    main() 