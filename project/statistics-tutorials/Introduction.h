
/**
 Demands for utility: A contractor is building an office complex and needs to plan water and electricity demands. He
 needs to hence decide the size of pipes, conduit and wires. After consulting the prospective tenants and examining
 historic data, the contractor decides tha thte demand for electricity will range from 1 million and 150 milion KWh
 per day and water demand will be between 4 and 200 ( in thousands of gallons per day ). All combinations of water
 and electricity and water demand are considered possible.

 A sample space denotes the set of outcomes consisting of various sequences of events. A random variable can be any
 event. This sample space is denoted by S, the sequence of events as s and the random variable as X. This random
 variable X is defined on the space S. This means, one cannot have a random variable like height of a
 combination of 2 animals greater than 1m in a sample space of birds. The random variable X should be defined in the
 sample space and hence the sample space must contain animals and not birds. Because of this dependancy, random varialbe
 X is denoted by X(s) where s belongs to S. In this case, s is any combination of two animals and the event is the
 addition of the height of the two animals.

 Sample space is 2^10 different sequence of heads and tails. Let X be the number of heads that are obtained with 10
 tosses of the coin. The possible values of X are 0 to 10. Here the random variable X ( number of heads )  is a
 real valued function that is defined in the sample space of 2^10 different sequences of heads and tails. It is
 possible to calculate the proability of the occurence of the random variable X in the sample space S. The
 distribution is the collection of all the probability i.e probability that 1 head occurs in 10 different tosses, 2
 heads occur in 10 different tosses.... 10 heads occur in 10 different tosses. This is a probability distribution
 function.

 A random variable is discrete, if there are at most countably many possible values for X. The distribution of a
 random variable is the collection of all probabilities for all subsets A of the real numbers. There is a countable
 set of number of heads when the coin is tossed 10 times. The countable sets is 0 to 10. And thats why the random
 variable is discrete. Subsequently, the distribution function of random variable is the probablity of the occurence
 of the individual sets and hence is also discrete. This is called discrete distribution. The sum of all the discrete
 distribution gives the overall probability of the occurence of that random varialbe in the sample space and is
 always equal to 1. Hence the individual distribution is just a distribution of the entire set, how the random
 variable would behave. In case it is a uniform distribtution, then all the elements will have the same probability
 and the sum of them would be 1. Ofcourse, if the set is smaller than the overall set, for example instead of taking
 only summing the probability of 5 defective items instead of a total of 15 defective items, the probability will be
 lesser than 1.

 Discrete distribution:
 A random variable is not discrete, when the random varialbe can take any value and not discrete values. In a rolling
 of a dice, there can only be discrete numbers and nothing in between.
 There are collection such as binonomial distribution and bernoulli distribution, that are based on the similiar
 phenomeman where the random variables take a finite collection.

 Uniform distribution:
 A discrete distribution which is taken from a set of an ordered consecutive set like a set of
 numbers from 1 to 100. A distribution cannot be assigned to an infinte sequence of possible values, but such
 a dsitribtuion can be assigned to any finite sequence. The uniform distribution represents the outcome of an
 experiment that is often described by saying that one of the integers 1,2,3 ....k is chosen at random. In the
 uniform distribution, the p.f of any value X=x is a constant value ( c) . The probability hance for a bounded interval
 [a,b] is equal to c*(b-a), where c is 1/(upperbound - lowerbound)
 A random variable X can have a uniform distribution ( uniform distribtion on the set of integers ) and the random
 variable X can be a even number.

 Bionomial distribution:
 A discrete distribution which is taken from a set of identical items like a set of n
 independent manufactured items from a assembly line. There are only two probabilities of the health of the item -
 either it is a defective item or its a non defective item. Thus a bionomial distribution represents the outcome of
 an experiment that is often described by saying that x out of n items chosen at random are defective. The random
 varialbe X will have a discrete distribution an the possible values of are X=x are {0,1,2.......n}
 A random varialbe X can have a bionomial distribution ( bionomial distribtuion on n articles ) and the random
 variable X can be the number of defective items.

 Bernoullis distribution:
 Bernoullis distribution is also a discrete distribution, where the number of items in the set is fixed to 1.

 Continous Distribution:
 Random variable that can assume every value in an interval (bounded or unbounded ) have continous distribution. A
 conntinous distribution can be characterized by its probability density function. We integrate the pdf of a random
 variable X over a set to compute the probability that X takes a value in that set. It is said that a random variable
 X has a continous distribution or that X is a continous random variable.

 Distribution function or cumulative distribution function:
 All distributions have a common characterization through their distribution function d.f / c.d.f The inverse of the df
 is called the quantile function and is useful for indicating where the probability is located in the distribution.
 Distribution of X is defined for every random variable X, regardless of whether the distribution of X is discrete or
 continous or mixed.

 --------------------
 Bivariate Distribution:
 In the bivariate distribution, the distribution is not only for a single random variable, but an additional random
 variable. This makes the go in table 2D. The principles for discrete distrubtion, continous distrubtion, and uniform
 distrubution applies here as well. The p.f, pdf and d.f are known as joint p.f ( between two disrete randome
 variable ), joint pdf ( between two continous random variable ) and joint d.f ( between two random variable ) .

 Discrete Joint Distribution

 Continous Joint Distribution

 Mixed Bivariate Distribution

 Bivariate Distribution function:

 pf - probability function for discrete distribution
 pdf - probability density function for continous distribution
 df - distribution function

*/
