"""
Lookup Table module. Based on https://jsbsim-team.github.io/jsbsim/FGTable_8cpp_source.html.
"""

import numpy as np

class LookUpTable:
    """ 
        Lookup Table for 1D and 2D data. 
        \n
        To create 1D table, `nCols` should be equal to 1. Please enter only `nRows`.
        \n
        Use left shift operator `<<` for fill the table. 
        \n\n
        >>> Note : It is not extrapolate the values from table. It means that
        if key is less than lowest key of table, it return lowest key value of table. It is
        same for highest key of table. 

        1D Table Example:
        -----------------

        >>> tbl = LookUpTable(3)
                # KEY # VALUES
        >>> tbl << 5 << 3 \\
                << 6 << 4 \\
                << 8 << 10
        >>> tbl.print_table()
        >>> print('Result:', tbl.get_value_1D(7.2))

        It prints :

        ... \\
        [[ 0.  0.]
         [ 5.  3.] \\
         [ 6.  4.] \\
         [ 8. 10.]] \\
        Result: 7.6000000000000005 \\
        ...

        2D Table Example:
        -----------------

        >>> tbl = LookUpTable(5,3)
               # KEY COL # VALUE COLS (3) 
        >>> tbl         << 500 << 1000 << 2500 \\ # KEY ROW
                << 10   << 5   << 15   << 54   \\ #    | 
                << 20   << 10  << 32   << 65   \\ #  VALUE
                << 40   << 28  << 56   << 67   \\ #   ROWS
                << 80   << 54  << 99   << 126  \\ #    |
                << 160  << 98  << 147  << 598    #    |
            
        >>> tbl.print_table()
        >>> print('Result:', tbl.get_value_2D(42.3, 789.3))

        It prints :
        
        ... \\
        [[   0.  500. 1000. 2500.]
         [  10.    5.   15.   54.] \\
         [  20.   10.   32.   65.] \\
         [  40.   28.   56.   67.] \\
         [  80.   54.   99.  126.] \\
         [ 160.   98.  147.  598.]] \\
        Result: 46.2613815 \\
        ...

    """

    def __init__(self, nRows = 1, nCols = 1) -> None:
        """
            Initialize table w.r.t related sizes.

            Arguments:
            ----------

            >>> nRows : Number of rows of table. Key line which is 0th row of the table is not included.
            >>> dtype : int

            >>> nCols : Number of columns of table. Key line which is 0th column of the table is not included.
            >>> dtype : int
        """
        assert type(nRows) == int and type(nCols) == int, "Please enter the table size as integer!"
        assert nRows > 0 and nCols > 0, "Please make sure the table size!"
        self._nRows, self._nCols = nRows, nCols
        if nCols == 1: # make sure table is 1D
            self._rowCounter = 1
            self._colCounter = 0
        else:
            self._rowCounter = 0
            self._colCounter = 1
        self._data = np.zeros((self._nRows + 1, self._nCols + 1), dtype=np.float32)
        self._lastRowIndex = self._lastColIndex = 2
    
    def __lshift__(self, n):
        """
            Filling the table with `<<` operator.
        """
        assert self._rowCounter <= self._nRows, "There is no space in table to fill. Check the table size!"
        self._data[self._rowCounter][self._colCounter] = np.float32(n)

        if (self._colCounter >= self._nCols):
            self._colCounter = 0
            self._rowCounter += 1
        else:
            self._colCounter += 1
        return self

    def get_value_1D(self, key):
        """
            Getting value from 1D table for `key`.

            Arguments :

            >>> key   : Key value for table which return corresponding value from table.
            >>> dtype : float
        """
        r = self._lastRowIndex

        # if key is not in the table, do not extrapolate. Return the end value of table.
        if (key <= self._data[1][0]):
            self._lastRowIndex = 2
            return self._data[1][1]
        elif (key >= self._data[self._nRows][0]):
            self._lastRowIndex = self._nRows
            return self._data[self._nRows][0]

        # if key is middle of the table, search it.

        while (r > 2           and self._data[r-1][0] > key): r -= 1
        while (r < self._nRows and self._data[r]  [0] < key): r += 1

        self._lastRowIndex = r

        span = self._data[r][0] - self._data[r-1][0]
        if (span != 0.0):
            factor = (key - self._data[r-1][0]) / span
            if (factor > 1.0) : factor = 1.0
        else:
            factor = 1.0

        value = factor * (self._data[r][1] - self._data[r-1][1]) + self._data[r-1][1]

        return value


    def get_value_2D(self, rowKey, colKey):
        """
            Getting value from 2D table for `rowKey` and `colKey`.

            Arguments :

            >>> rowKey   : Key value for row of the table which return corresponding value from table.
            >>> dtype    : float

            >>> colKey   : Key value for column of the table which return corresponding value from table.
            >>> dtype    : float
        """

        r, c = self._lastRowIndex, self._lastColIndex

        while (r > 2           and self._data[r-1][0] > rowKey): r -= 1
        while (r < self._nRows and self._data[r]  [0] < rowKey): r += 1

        while (c > 2           and self._data[0][c-1] > colKey): c -= 1
        while (c < self._nCols and self._data[0][c]   < colKey): c += 1

        self._lastRowIndex, self._lastColIndex = r, c

        rFactor = (rowKey - self._data[r-1][0]) / (self._data[r][0] - self._data[r-1][0])
        cFactor = (colKey - self._data[0][c-1]) / (self._data[0][c] - self._data[0][c-1])

        if (rFactor > 1.0): rFactor = 1.0
        elif (rFactor < 0.0): rFactor = 0.0

        if (cFactor > 1.0): cFactor = 1.0
        elif (cFactor < 0.0): cFactor = 0.0

        col1temp = rFactor * (self._data[r][c-1] - self._data[r-1][c-1]) + self._data[r-1][c-1]
        col2temp = rFactor * (self._data[r][c]   - self._data[r-1][c]  ) + self._data[r-1][c]

        value = col1temp + cFactor * (col2temp - col1temp)
        
        return value

    def print_table(self):
        """
            Printing the table.
        """
        print(self._data)
        

if __name__ == "__main__":
    tbl = LookUpTable(5,3)
    tbl         << 500 << 1000 << 2500 \
        << 10   << 5   << 15   << 54   \
        << 20   << 10  << 32   << 65   \
        << 40   << 28  << 56   << 67   \
        << 80   << 54  << 99   << 126  \
        << 160  << 98  << 147  << 598
     
    tbl.print_table()
    print('Result:', tbl.get_value_2D(42.3, 789.3))

