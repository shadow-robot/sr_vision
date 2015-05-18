## benchmarking
    This package contains a testing program that return the score of several algorithms in order to order them.
    The executive file is score.py located in /script and secondary dependancies files are in the /src folder.
    Score is calculated with the difference of segments found by the tested algorithm and the distance between points that are wrong. The smallest score is the best (0=perfect concordance).
    
## drawing
    Comparison is based upon a reference dataset. For now composed by a basic test dataset and a noise test one, with images drawn thanks to PIL. Points coordinates can be stored in .seg files in DataSet/Reference_seg with the _write_seg_file method but in fact it isn't useful. Will be completed by textured and real pictures.


## read_file
    For now useless but will be later with the Berkeley's dataset.
