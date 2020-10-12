import argparse
import json
import numpy as np
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def get_benchmark(args: dict) -> dict:
    with open(args["in"], "r") as file:
        report = json.load(file)
        benchmark = {
            "bm_vector_construction": 
                [result for result in report["benchmarks"] if result["name"].startswith("bm_vector_construction")],
            "bm_octree_construction": 
                [result for result in report["benchmarks"] if 
                    result["name"].startswith("bm_octree_construction") and result["name"].endswith("/4/21")],
            "bm_vector_range_search": 
                [result for result in report["benchmarks"] if result["name"].startswith("bm_vector_range_search")],
            "bm_octree_range_search": 
                [result for result in report["benchmarks"] if result["name"].startswith("bm_octree_range_search")],
            "bm_vector_knn_search": 
                [result for result in report["benchmarks"] if result["name"].startswith("bm_vector_knn_search")],
            "bm_octree_knn_search": 
                [result for result in report["benchmarks"] if result["name"].startswith("bm_octree_knn_search")]
        }

        return benchmark

def from_nanoseconds_to_seconds(duration: int) -> int:
    return duration / 1000000000

def plot(benchmark: dict):
    # execution_time(number_of_searches, number_of_points) = 
    #   construction_time(number_of_points) + number_of_searches*search_time(number_of_points)
    fig: plt.Figure = plt.figure()
    range_search_ax: Axes3D = fig.add_subplot(1, 2, 1, projection="3d")
    knn_search_ax:   Axes3D = fig.add_subplot(1, 2, 2, projection="3d")
    
    number_of_points: np.ndarray = np.array([int(result["name"].split("/")[1]) for result in benchmark["bm_vector_construction"]])
    number_of_searches: np.ndarray = np.arange(0, 65536, 1024)
    vector_construction_time = [from_nanoseconds_to_seconds(int(result["cpu_time"])) for result in benchmark["bm_vector_construction"]]
    octree_construction_time = [from_nanoseconds_to_seconds(int(result["cpu_time"])) for result in benchmark["bm_octree_construction"]]
    vector_range_search_time = [from_nanoseconds_to_seconds(int(result["cpu_time"])) for result in benchmark["bm_vector_range_search"]]
    octree_range_search_time = [from_nanoseconds_to_seconds(int(result["cpu_time"])) for result in benchmark["bm_octree_range_search"]]
    vector_knn_search_time   = [from_nanoseconds_to_seconds(int(result["cpu_time"])) for result in benchmark["bm_vector_knn_search"]]
    octree_knn_search_time   = [from_nanoseconds_to_seconds(int(result["cpu_time"])) for result in benchmark["bm_octree_knn_search"]]

    number_of_points, number_of_searches = np.meshgrid(number_of_points, number_of_searches)
    vector_range_search_z = np.zeros(number_of_points.shape)
    octree_range_search_z = np.zeros(number_of_points.shape)
    vector_knn_search_z   = np.zeros(number_of_points.shape)
    octree_knn_search_z   = np.zeros(number_of_points.shape)
    for i in range(number_of_points.shape[1]):
        vector_range_search_z[:,i] = vector_construction_time[i] + vector_range_search_time[i] * number_of_searches[:,i]
        octree_range_search_z[:,i] = octree_construction_time[i] + octree_range_search_time[i] * number_of_searches[:,i]
        vector_knn_search_z[:,i]   = vector_construction_time[i] + vector_knn_search_time[i]   * number_of_searches[:,i]
        octree_knn_search_z[:,i]   = octree_construction_time[i] + octree_knn_search_time[i]   * number_of_searches[:,i]
        
    vector_color = "y"
    octree_color = "b"

    range_search_ax.plot_surface(number_of_points, number_of_searches, octree_range_search_z, color=octree_color)
    range_search_ax.plot_surface(number_of_points, number_of_searches, vector_range_search_z, color=vector_color)
    knn_search_ax.plot_surface(number_of_points, number_of_searches, octree_knn_search_z, color=octree_color)
    knn_search_ax.plot_surface(number_of_points, number_of_searches, vector_knn_search_z, color=vector_color)

    vector_patch = mpatches.Patch(color=vector_color, label="vector")
    octree_patch = mpatches.Patch(color=octree_color, label="octree")
    range_search_ax.legend(handles=[vector_patch, octree_patch])
    knn_search_ax.legend(handles=[vector_patch, octree_patch])

    range_search_ax.set_title("Range search")
    knn_search_ax.set_title("K nearest neighbors")

    range_search_ax.set_xlabel("Number of points")
    knn_search_ax.set_xlabel("Number of points")
    range_search_ax.set_ylabel("Number of searches")
    knn_search_ax.set_ylabel("Number of searches")
    range_search_ax.set_zlabel("Execution time (s)")
    knn_search_ax.set_zlabel("Execution time (s)")

    fig.suptitle("Octree vs. Vector\n(including data structure construction time)")

    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plot benchmark results")
    parser.add_argument("--in", dest="in", help="json file of benchmark ouput")

    args = parser.parse_args()
    args = vars(args)

    benchmark: dict = get_benchmark(args)
    plot(benchmark)
