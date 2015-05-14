## benchmarking
    This package contains a testing program that return the score of several algorithms in order to order them.
    Based upon a reference dataset. For now composed by a basic test dataset and a noise test one, with images drawn thanks to SimpleCV. Points coordinates are stored in .seg files in DataSet/Reference_seg. Will be completed by textured and real pictures.
    Score is calculated with the difference of segments found by the tested algorithm and the distance between points that are wrong. The smallest score is the best (0=perfect concordance).


