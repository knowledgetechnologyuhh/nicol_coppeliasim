# Copyright (c) 2023. Lennart Clasmeier. GNU General Public License https://www.gnu.org/licenses/gpl-3.0.html.

import threading
from pyrep import PyRep

class BackgroundStepping(threading.Thread):
    def __init__(self, pr_env:PyRep, name="Background Stepping"):
        threading.Thread.__init__(self)
        self.stop_flag = False
        self.pr = pr_env


    def run(self):
        while not self.stop_flag:
            self.pr.step()
        print("stepping-loop stopped")

    def stop(self, timeout=None):
        """ Stop the thread. """
        self.stop_flag = True
        threading.Thread.join(self, timeout)
