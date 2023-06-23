

import math
import random
import numpy as np


from .utils import fast_min, fast_max


class Mutations:

    @staticmethod
    def mutations_dict():
        return {
            'uniform_by_x': Mutations.uniform_by_x(),
            'uniform_by_center': Mutations.uniform_by_center(),
            'gauss_by_center': Mutations.gauss_by_center(),
            'gauss_by_x': Mutations.gauss_by_x(),
        }
    @staticmethod
    def mutations_discrete_dict():
        return {
            'uniform_discrete': Mutations.uniform_discrete()
        }


    @staticmethod
    def uniform_by_x():
        
        def func(x: float, left: float, right: float):
            alp = fast_min(x - left, right - x)
            return random.uniform(x - alp, x + alp)
        return func

    @staticmethod
    def uniform_by_center():
        
        def func(x: float, left: float, right: float):
            return random.uniform(left, right)
        
        return func

    @staticmethod
    def gauss_by_x(sd: float = 0.3):
        """
        gauss mutation with x as center and sd*length_of_zone as std
        """
        def func(x: float, left: float, right: float):
            std = sd * (right - left)
            return fast_max(left, fast_min(right, np.random.normal(loc = x, scale = std)))
        
        return func

    @staticmethod
    def gauss_by_center(sd: float = 0.3):
        """
        gauss mutation with (left+right)/2 as center and sd*length_of_zone as std
        """
        def func(x: float, left: float, right: float):
            std = sd * (right - left)
            return fast_max(left, fast_min(right, np.random.normal(loc =(left + right) * 0.5, scale = std)))
        
        return func

    @staticmethod
    def uniform_discrete():
        def func(x: int, left: int, right: int):
            return random.randint(left, right)
        return func

