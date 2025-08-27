"""
Interactive Encoder Analysis Script
Provides a GUI interface for creating custom visualizations of encoder data.
"""

import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from pathlib import Path
import os

class EncoderAnalysisGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Interactive Encoder Analysis")
        self.root.geometry("1200x800")
        
        # Data storage
        self.df = None
        self.figure = None
        
        # Initialize GUI
        self.setup_gui()
        self.load_default_data()
        
    def setup_gui(self):
        """Set up the GUI layout."""
        # Create main frames
        control_frame = ttk.Frame(self.root, padding="10")
        control_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        plot_frame = ttk.Frame(self.root, padding="10")
        plot_frame.grid(row=0, column=1, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Configure grid weights
        self.root.columnconfigure(1, weight=1)
        self.root.rowconfigure(0, weight=1)
        
        # --- Control Frame Widgets ---
        
        # File loading
        file_frame = ttk.LabelFrame(control_frame, text="Data Loading", padding="10")
        file_frame.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E))
        
        self.file_label = ttk.Label(file_frame, text="No file loaded.")
        self.file_label.grid(row=0, column=0, sticky=tk.W)
        
        load_button = ttk.Button(file_frame, text="Load Data", command=self.load_data)
        load_button.grid(row=0, column=1, sticky=tk.E)
        
        # Plot configuration
        plot_config_frame = ttk.LabelFrame(control_frame, text="Plot Configuration", padding="10")
        plot_config_frame.grid(row=1, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=10)
        
        # Variable selection
        ttk.Label(plot_config_frame, text="X-axis Variable:").grid(row=0, column=0, sticky=tk.W)
        self.x_var = tk.StringVar()
        self.x_var_combo = ttk.Combobox(plot_config_frame, textvariable=self.x_var)
        self.x_var_combo.grid(row=0, column=1, sticky=(tk.W, tk.E))
        
        ttk.Label(plot_config_frame, text="Y-axis Variable:").grid(row=1, column=0, sticky=tk.W)
        self.y_var = tk.StringVar()
        self.y_var_combo = ttk.Combobox(plot_config_frame, textvariable=self.y_var)
        self.y_var_combo.grid(row=1, column=1, sticky=(tk.W, tk.E))
        
        ttk.Label(plot_config_frame, text="Color By:").grid(row=2, column=0, sticky=tk.W)
        self.color_var = tk.StringVar()
        self.color_var_combo = ttk.Combobox(plot_config_frame, textvariable=self.color_var)
        self.color_var_combo.grid(row=2, column=1, sticky=(tk.W, tk.E))

        # --- REVISED: Controlled variable section ---
        ttk.Label(plot_config_frame, text="Controlled Variable:").grid(row=3, column=0, sticky=tk.W)
        self.control_var = tk.StringVar()
        self.control_var_combo = ttk.Combobox(plot_config_frame, textvariable=self.control_var)
        self.control_var_combo.grid(row=3, column=1, sticky=(tk.W, tk.E))
        self.control_var_combo.bind("<<ComboboxSelected>>", self.update_fixed_value_options)

        ttk.Label(plot_config_frame, text="Fixed Value:").grid(row=4, column=0, sticky=tk.W)
        self.fixed_val = tk.StringVar()
        self.fixed_val_combo = ttk.Combobox(plot_config_frame, textvariable=self.fixed_val)
        self.fixed_val_combo.grid(row=4, column=1, sticky=(tk.W, tk.E))
        # --- END REVISION ---

        # Plot type selection
        ttk.Label(plot_config_frame, text="Plot Type:").grid(row=5, column=0, sticky=tk.W)
        self.plot_type = tk.StringVar(value='scatterplot')
        plot_type_combo = ttk.Combobox(plot_config_frame, textvariable=self.plot_type, 
                                       values=['scatterplot', 'boxplot', 'violinplot', 'lineplot', 'histogram'])
        plot_type_combo.grid(row=5, column=1, sticky=(tk.W, tk.E))
        
        # Plot options
        options_frame = ttk.LabelFrame(control_frame, text="Plot Options", padding="10")
        options_frame.grid(row=2, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=10)
        
        self.legend_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(options_frame, text="Show Legend", variable=self.legend_var).pack(anchor=tk.W)
        
        self.error_line_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(options_frame, text="Show Zero Error Line", variable=self.error_line_var).pack(anchor=tk.W)
        
        # Action buttons
        action_frame = ttk.Frame(control_frame, padding="10")
        action_frame.grid(row=3, column=0, columnspan=2)
        
        ttk.Button(action_frame, text="Create Plot", command=self.create_plot).pack(side=tk.LEFT, padx=5)
        ttk.Button(action_frame, text="Save Plot", command=self.save_plot).pack(side=tk.LEFT, padx=5)
        ttk.Button(action_frame, text="Clear Plot", command=self.clear_plot).pack(side=tk.LEFT, padx=5)
        
        # --- Plot Frame Setup ---
        self.figure = Figure(figsize=(8, 6), dpi=100)
        self.canvas = FigureCanvasTkAgg(self.figure, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
    def load_default_data(self):
        """Load the default CSV file if it exists."""
        # Get the directory where this script is located
        script_dir = Path(__file__).parent
        default_file = script_dir / "encoder_data.csv"
        
        if default_file.exists():
            self.load_file(default_file)
        else:
            # Fallback to current directory for backward compatibility
            fallback_file = Path("encoder_data.csv")
            if fallback_file.exists():
                self.load_file(fallback_file)
            
    def load_data(self):
        """Open a file dialog to load a CSV file."""
        filepath = filedialog.askopenfilename(
            title="Open CSV File",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
        )
        if filepath:
            self.load_file(Path(filepath))
            
    def load_file(self, filepath):
        """Load and process the selected CSV file."""
        try:
            self.df = pd.read_csv(filepath)
            self.file_label.config(text=f"Loaded: {filepath.name}")
            
            # Update combobox options
            self.update_variable_options()
        except Exception as e:
            messagebox.showerror("Error", f"Failed to load file: {str(e)}")
            
    def update_variable_options(self):
        """Update the variable selection comboboxes with column names."""
        if self.df is not None:
            columns = list(self.df.columns)
            
            # Add a 'None' option for optional selections
            columns_with_none = [''] + columns
            
            self.x_var_combo['values'] = columns
            self.y_var_combo['values'] = columns
            self.color_var_combo['values'] = columns_with_none
            
            # --- REVISED: Update control variable options ---
            self.control_var_combo['values'] = columns_with_none
            self.control_var.set('')
            self.fixed_val.set('')
            self.fixed_val_combo['values'] = []
            # --- END REVISION ---

    def update_fixed_value_options(self, event=None):
        """
        --- NEW FUNCTION ---
        Update the fixed value combobox based on the selected controlled variable.
        """
        control_var = self.control_var.get()
        if self.df is not None and control_var:
            try:
                unique_values = sorted(self.df[control_var].unique())
                self.fixed_val_combo['values'] = unique_values
                self.fixed_val.set(unique_values[0] if unique_values else '')
            except KeyError:
                self.fixed_val_combo['values'] = []
                self.fixed_val.set('')
        else:
            self.fixed_val_combo['values'] = []
            self.fixed_val.set('')

    def create_plot(self):
        """Create a plot based on the current selections."""
        if self.df is None:
            messagebox.showerror("Error", "No data loaded.")
            return
            
        x_var = self.x_var.get()
        y_var = self.y_var.get()
        
        if not x_var or not y_var:
            messagebox.showerror("Error", "Please select X and Y axis variables.")
            return
            
        self.figure.clear()
        ax = self.figure.add_subplot(111)
        
        # --- REVISED: Filter data based on controlled variable ---
        plot_df = self.df.copy()
        control_var = self.control_var.get()
        fixed_val_str = self.fixed_val.get()
        
        if control_var and fixed_val_str:
            try:
                # Convert fixed value to the correct dtype
                dtype = self.df[control_var].dtype
                
                # Handle different data types properly
                if pd.api.types.is_numeric_dtype(dtype):
                    # For numeric types, convert directly
                    fixed_val = pd.to_numeric(fixed_val_str)
                elif pd.api.types.is_datetime64_any_dtype(dtype):
                    # For datetime types
                    fixed_val = pd.to_datetime(fixed_val_str)
                else:
                    # For string/object types, keep as string
                    fixed_val = str(fixed_val_str)
                
                plot_df = self.df[self.df[control_var] == fixed_val]
                ax.set_title(f'{y_var} vs. {x_var}\n(where {control_var} = {fixed_val})')
            except (ValueError, KeyError) as e:
                messagebox.showerror("Error", f"Invalid fixed value or variable: {e}")
                return
        else:
            ax.set_title(f'{y_var} vs. {x_var}')
        # --- END REVISION ---
        
        color_var = self.color_var.get() if self.color_var.get() else None
        
        try:
            plot_type = self.plot_type.get()
            
            # Set high contrast color palette
            colors = ['#FF0000', '#0000FF', '#00FF00', '#FF8000', '#800080', 
                     '#00FFFF', '#FFFF00', '#FF00FF', '#000000', '#808080']
            
            if plot_type == 'scatterplot':
                sns.scatterplot(data=plot_df, x=x_var, y=y_var, hue=color_var, ax=ax, 
                               palette=colors if color_var else None, s=60)
            elif plot_type == 'boxplot':
                sns.boxplot(data=plot_df, x=x_var, y=y_var, hue=color_var, ax=ax,
                           palette=colors if color_var else None)
            elif plot_type == 'violinplot':
                sns.violinplot(data=plot_df, x=x_var, y=y_var, hue=color_var, ax=ax,
                              palette=colors if color_var else None)
            elif plot_type == 'lineplot':
                sns.lineplot(data=plot_df, x=x_var, y=y_var, hue=color_var, ax=ax, 
                            errorbar=None, palette=colors if color_var else None, linewidth=2)
            elif plot_type == 'histogram':
                sns.histplot(data=plot_df, x=x_var, hue=color_var, multiple="stack", ax=ax,
                            palette=colors if color_var else None)
            
            self.finalize_plot(ax, x_var, y_var, color_var)
            self.canvas.draw()
            
        except Exception as e:
            messagebox.showerror("Plotting Error", f"An error occurred: {str(e)}")
            
    def finalize_plot(self, ax, x_var, y_var, color_var):
        """Final touches for the plot."""
        ax.set_xlabel(x_var.replace('_', ' ').title(), fontsize=12, fontweight='bold')
        ax.set_ylabel(y_var.replace('_', ' ').title(), fontsize=12, fontweight='bold')
        ax.grid(True, which='both', linestyle='--', linewidth=0.5, alpha=0.7)
        
        # Set background color for better contrast
        ax.set_facecolor('#F8F8F8')
        
        if self.legend_var.get() and color_var:
            legend = ax.legend(frameon=True, fancybox=True, shadow=True)
            legend.get_frame().set_facecolor('white')
            legend.get_frame().set_alpha(0.9)
            
        if self.error_line_var.get() and y_var == 'error':
            ax.axhline(y=0, color='red', linestyle='--', alpha=0.8, linewidth=2)
            
        # Improve tick labels
        ax.tick_params(axis='both', which='major', labelsize=10)
        
        # Set title with better formatting
        if ax.get_title():
            ax.set_title(ax.get_title(), fontsize=14, fontweight='bold', pad=20)
            
    def save_plot(self):
        """Save the current plot to file."""
        if self.figure is None or not self.figure.get_axes():
            messagebox.showerror("Error", "No plot to save.")
            return
            
        filename = filedialog.asksaveasfilename(
            title="Save plot",
            defaultextension=".png",
            filetypes=[("PNG files", "*.png"), ("PDF files", "*.pdf"), 
                      ("SVG files", "*.svg"), ("All files", "*.*")]
        )
        
        if filename:
            try:
                self.figure.savefig(filename, dpi=300, bbox_inches='tight')
                messagebox.showinfo("Success", f"Plot saved to {filename}")
            except Exception as e:
                messagebox.showerror("Error", f"Failed to save plot: {str(e)}")
                
    def clear_plot(self):
        """Clear the current plot."""
        if self.figure:
            self.figure.clear()
            self.canvas.draw()

def main():
    """Main function to run the GUI."""
    root = tk.Tk()
    
    # Set up ttk styles
    style = ttk.Style(root)
    try:
        # For a more modern look if available (e.g., on Windows 10+)
        style.theme_use('vista') 
    except tk.TclError:
        # Fallback to a default theme
        style.theme_use('clam')

    app = EncoderAnalysisGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()