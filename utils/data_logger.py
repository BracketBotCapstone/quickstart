import matplotlib.pyplot as plt
import pandas as pd


class DataLogger:
    def __init__(self):
        """
        Initialize the DataLogger with an empty dictionary to store data.
        """
        self.data = {}
    
    def log(self, **kwargs):
        """
        Log any number of variables provided as keyword arguments.
        """
        for key, value in kwargs.items():
            if key not in self.data:
                self.data[key] = []
            self.data[key].append(value)
    
    def plot(self, x_key='time', y_keys=None):
        """
        Plot the logged data.
        :param x_key: The key to use for the x-axis (default is 'time').
        :param y_keys: A list of keys to plot against x_key. If None, all keys except x_key are plotted.
        """
        if y_keys is None:
            y_keys = [key for key in self.data.keys() if key != x_key]
        
        # Calculate number of rows needed for 2 columns
        n_plots = len(y_keys)
        n_rows = (n_plots + 1) // 2  # Round up division
        
        # Create subplot grid
        fig, axes = plt.subplots(n_rows, 2, figsize=(12, 4*n_rows))
        axes = axes.flatten()  # Flatten to make indexing easier
        
        # Plot each variable
        for i, y_key in enumerate(y_keys):
            ax = axes[i]
            ax.plot(self.data[x_key], self.data[y_key], label=y_key)
            ax.set_xlabel(x_key)
            ax.set_ylabel(y_key)
            ax.set_title(f'{y_key} vs {x_key}')
            ax.legend()
            
        # Remove any empty subplots
        for i in range(n_plots, len(axes)):
            fig.delaxes(axes[i])
            
        plt.tight_layout()
        plt.savefig('plots.png')

    def to_csv(self, filename, max_num=None):
        """
        Save the logged data to a CSV file.
        :param filename: Name of the CSV file to save to
        :param max_num: Maximum number of most recent entries to save. If None, saves all entries.
        """
        df = pd.DataFrame(self.data)
        if max_num is not None:
            df = df.tail(max_num)
        df.to_csv(filename, index=False)

    def from_csv(self, filename):
        """
        Load logged data from a CSV file.
        :param filename: Name of the CSV file to load from
        """
        df = pd.read_csv(filename)
        self.data = df.to_dict(orient='list')