"""Simple timing utility for performance measurement"""

import time
from contextlib import contextmanager
from typing import Optional


class Timer:
    """Context manager for timing code execution"""
    
    def __init__(self, name: str = "Operation", verbose: bool = True):
        """
        Initialize timer.
        
        Parameters:
        -----------
        name : str
            Name of the operation being timed
        verbose : bool
            Whether to print timing information
        """
        self.name = name
        self.verbose = verbose
        self.start_time: Optional[float] = None
        self.elapsed: float = 0.0
    
    def __enter__(self):
        """Start timer on entering context"""
        self.start_time = time.time()
        if self.verbose:
            print(f"  {self.name}... ", end="", flush=True)
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Stop timer on exiting context"""
        self.elapsed = time.time() - self.start_time
        if self.verbose:
            print(f"✓ ({self.elapsed:.2f}s)")
    
    def __str__(self) -> str:
        return f"{self.name}: {self.elapsed:.2f}s"


@contextmanager
def time_block(name: str = "Operation", verbose: bool = True):
    """
    Context manager for timing code blocks.
    
    Usage:
    ------
    with time_block("Loading data"):
        data = load_data()
    """
    timer = Timer(name, verbose)
    with timer:
        yield timer
