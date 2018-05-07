#!/usr/bin/env python2

import pandas as pd
import rospy
import traceback
import matplotlib
matplotlib.use('agg')
import os
import errno


class Statistics(object):
    """
    Collecting arbitrary statistics data in simulation steps
    
    update_step() has to be called every round.
    If the latest changes (current row) should be included in the statistics
    finalise() has to be called before exports or descriptive statistics
    
    Objects holds current and last row
    
    Python pandas framework is used to create more advanced statistics
    """

    STEP_COLUMN_NAME = 'step'

    def __init__(self, add_step_column=True):
        """
        :param add_step_column: add a step column on update and finalise if True
        """
        self._current_step = 0
        self._rows = []

        self._add_step_column = add_step_column

        self._current_step_row = {}
        self._new_step_row(step=self._current_step)

    def _new_step_row(self, step):
        """
        Create new row for a new step
        :param step: step number 
        """
        self._last_step_row = self._current_step_row
        self._current_step_row = {}

        if self._add_step_column:
            self._current_step_row[Statistics.STEP_COLUMN_NAME] = step

    def update_step(self, step):
        """
        Initialise the next simulation step round, automatically finalises the current row
        :param step: step number
        """
        if step > self._current_step:
            self._rows.append(self._current_step_row)
            self._new_step_row(step=step)
        self._current_step = step

    def increment_last_value(self, key, value):
        """
        Increment the last value of this key by the given value
        If last value is None, will be calculated as 0
        :param key: key(column)
        :param value: increment value
        """
        try:
            last = self._last_step_row[key]
        except KeyError:
            last = 0
        self._current_step_row[key] = last + value

    def increment_current_value(self, key, value):
        """
        Increment the current value of this key by the given value
        If current value is None, will try to get the last value as reference, 
        if this is also not available it uses 0
        :param key: key(column)
        :param value: increment value
        """
        try:
            last = self._current_step_row[key]
        except KeyError:
            try:
                last = self.get_last_value(key=key)
            except KeyError:
                last = 0
        self._current_step_row[key] = last + value

    def add_value(self, key, value):
        """
        Add value to the data collection
        :param key: key(column)
        :param value: new value
        :return: 
        """
        self._current_step_row[key] = value

    def get_last_value(self, key):
        """
        get the last(before last update_step()) value of a key
        :param key: key(column)
        :return: last value
        """
        return self._last_step_row[key]

    def get_current_value(self, key):
        """
        get the current (after last update_step()) value of a key
        :param key: key(column)
        :return: current value
        """
        return self._current_step_row[key]

    def finalise(self):
        """
        finalise the currently collected simulation step row and increases the step counter
        :return: 
        """
        self._rows.append(self._current_step_row)
        self._new_step_row(step=self._current_step+1)

    def get_panda_frame(self):
        """
        get all rows as pandas.DataFrame        
        :return: DataFrame with all data
        """
        df = pd.DataFrame(self._rows)
        if self._add_step_column:
            df.set_index(Statistics.STEP_COLUMN_NAME)

        return df

    def get_descriptive_stats(self):
        """
        get descriptive statistics of all rows max, min, mean ...
        call finalise() before if last added data should be included
        :return: statistics DataFrame
        """
        df = self.get_panda_frame()
        return df.describe()

    def _make_dir_available(self, filename):
        if not os.path.exists(os.path.dirname(filename)):
            try:
                os.makedirs(os.path.dirname(filename))
            except OSError as exc:  # Guard against race condition
                if exc.errno != errno.EEXIST:
                    raise

    def write_to_csv(self, filename, append=False):
        """
        Write collected data to csv
        call finalise() before if last added data should be included
        :param filename: name without extension
        """
        try:
            filename = filename + '.csv'
            df = self.get_panda_frame()
            if append and os.path.exists(filename):
                with open(filename, 'a') as f:
                    df.to_csv(f, header=False, index=False, encoding='utf-8')
            else:
                self._make_dir_available(filename)
                df.to_csv(filename, index=False, encoding='utf-8')
        except Exception:
            rospy.logerr("Creating CSV statistics file failed for filename '%s'  with %s", filename, traceback.format_exc())

    def write_to_plot(self, filename):
        """
        Write collected data to png plot
        call finalise() before if last added data should be included
        :param filename: name without extension
        """
        try:
            filename = filename + '.png'
            self._make_dir_available(filename)
            df = self.get_panda_frame()
            ax = df.plot()
            fig = ax.get_figure()
            fig.savefig(filename)
        except Exception:
            rospy.logerr("Creating statistics plot file failed for filename '%s'  with %s", filename, traceback.format_exc())


if __name__ == '__main__':

    # Example statistics demonstration below

    stats = Statistics()

    for i in range(10):
        stats.update_step(i)
        stats.add_value('a', i)
        stats.add_value('b', i / 2.0)
        stats.increment_last_value('c', 1)
        stats.increment_current_value('d', 1)
        stats.increment_current_value('d', 1)

    stats.finalise()
    print(stats.get_panda_frame())

    print(stats.get_descriptive_stats())

    stats.write_to_csv('test')
    stats.write_to_plot('test')
