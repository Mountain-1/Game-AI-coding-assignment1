# Project:  CS 330
# Program:  Dynamic movement, 4 Plot
# Purpose:  Plot a movement trajectories generated by Dynamic movement function.
# Author:   Mikel D. Petty, Ph.D., 256-824-6140, pettym@uah.edu
# Created:  2020-1-30
# Modified: 2022-2-4

rm(list = ls())      # Clear workspace
options(scipen=999)  # Suppress scientific notation

# Set execution control parameters.

computer <- 2        # Computer code is running on:  1=Alpha (OKT N353), 2=Jay Sebastian
scenario <- 27       # Scenario to plot
animate  <- FALSE    # Animate?  TRUE=animate, FALSE=full trajectory only

# Initialize file paths and names.

if (computer == 1) {
  work.path     <- "C:/Users/mpetty/Desktop/Working, CS 330/"
  source.path   <- "C:/Users/mpetty/Desktop/Movement, Dynamic 22S/"
} else {
  work.path     <- "Z:/UAH/CS330 Fall 2022/Source Material/R Code/"
  source.path   <- "Z:/UAH/CS330 Fall 2022/Source Material/R Code/"
}

#trajectory.file <- paste(work.path, "CS 330, Dynamic ", scenario, ", Trajectory data.txt", sep="")
trajectory.file <- paste(work.path, "CS3302.txt", sep="")
# Load support functions; initialize global and scenario variables.

source(paste(source.path, "CS 330, Dynamic movement, 1 Support v8.r", sep=""))
source(paste(source.path, "CS 330, Dynamic movement, 2 Initialize v12.r", sep=""))

# Set plot control constants.

POSITION        <- 1
VELOCITY        <- 2
LINEAR          <- 3
ORIENTATION     <- 4
PATHS           <- 5
COLLISIONS      <- 6

plot.names      <- c("position", "velocity", "linear", "orientation", "paths",  "collisions")
plot.colors     <- c("red",      "green",    "blue",   "goldenrod3",  "gray32", "orange2")

steering.names  <- c("Continue", "Stop",   "Align",  "Face target", "Face movement", "Seek",             "Flee",
                     "Arrive",   "Pursue", "Wander", "Follow path", "Separate",       "Avoid collisions", "Swirl")
                     
if (scenario == 28) { steering.names <- paste("Unknown", sample(21:34, replace=FALSE)) }
if (scenario == 29) { steering.names <- paste("Unknown", sample(41:54, replace=FALSE)) }

# Read trajectory data from file into data frame.

trajectory.all  <- read.csv(file=trajectory.file, header=FALSE, sep=",", skip=0)

# Dynamic trajectory file records each have 10 fields, all relating to a single character.
# All fields are numeric, unless otherwise noted.
#  1 simultion time
#  2 id
#  3 position x
#  4 position z
#  5 velocity x
#  6 velocity z
#  7 linear x
#  8 linear z
#  9 orientation
# 10 steering behavior
# 11 collision status (logical)

# Plot trajectory frame.

plot.frame <- function(f, frame.time, trajectory.frame) {
 
  plot.file <- paste(work.path, "CS 330, Dynamic ", scenario, " ", f, ".png", sep="")

  png(file=plot.file, width=1350, height=1350, res=300)  # Open plot file
  par(mar=c(2.5, 3.0, 1.5, 1.0))                         # Set margins bottom, left, top, right

  axis.lims.x  <- c(-100, 100)
  axis.ticks.x <- seq(from=axis.lims.x[1], to=axis.lims.x[2], by=20)
  axis.lims.y  <- c(100, -100)
  axis.ticks.y <- seq(from=axis.lims.y[1], to=axis.lims.y[2], by=-20)

  plot(NULL, xlim=axis.lims.x, ylim=axis.lims.y, xaxt="n", xaxt="n", yaxt="n", xlab="", ylab="", main="")

  axis(side=1, at=axis.ticks.x, labels=axis.ticks.x, cex.axis=0.80, mgp=c(3, 0.50, 0))
  axis(side=2, at=axis.ticks.y, labels=axis.ticks.y, cex.axis=0.80, mgp=c(3, 0.75, 0), las=1)

  title.1    <- "Dynamic"
  title.2    <- scenario
  title.3    <- ifelse(physics, "HS", "NE1")
  title.4    <- num.width(delta.time, 1, 2)
  title.5    <- paste(c("P", "V", "L", "O", "H", "C")[which(plot.what)], collapse="")
  title.main <- paste(title.1, title.2, title.3, title.4, title.5)

  title.x    <- expression(italic(x))
  title.y    <- expression(italic(z))

  title(main=title.main, line=0.50, cex.main=0.75)
  title(xlab=title.x, line=1.25, cex.lab=1.0, family="serif")
  title(ylab=title.y, line=2.00, cex.lab=1.0, family="serif")

  if (plot.cross.refs) {
    abline(h=0, col="gray", lty="dashed")
    abline(v=0, col="gray", lty="dashed")
  }

  legend("bottomright", legend=plot.names, col=plot.colors, lty=1, bty="n", cex=0.50)

  if (animate) {
    frame.text <- paste("frame =", f, "time =", frame.time)
    text(axis.lims.x[1], axis.lims.y[1], frame.text, col="black", cex=0.50, pos=4)
  }

  if (plot.what[5]) {  # Plot paths
    for (i in 1:paths) {
      lines(type="l", x=Path[[i]]$x, y=Path[[i]]$y, col=plot.colors[PATHS], lty="dashed", lwd=1.00)
      points(x=Path[[i]]$x, y=Path[[i]]$y, pch=20, col=plot.colors[PATHS])
      text(x=Path[[i]]$x[1], y=Path[[i]]$y[1], "path", col=plot.colors[PATHS], cex=0.50, pos=ifelse(i == 1, 2, 4))
      text(x=Path[[i]]$x, y=Path[[i]]$y, num.width(Path[[i]]$param, 1, 2), col=plot.colors[PATHS], cex=0.50, pos=1)
    }
  }

  for (i in 1:characters) {
    trajectory <- subset(trajectory.frame, trajectory.frame[,2] == Character[[i]]$id)

    time.steps <- length(trajectory[,1])
    for (j in 1:time.steps) {
      if (plot.what[2]) {  # Plot character's velocity
        vel.vec.x <- c(trajectory[j, 3], trajectory[j, 3] + (trajectory[j, 5] * plot.scale[1]))
        vel.vec.z <- c(trajectory[j, 4], trajectory[j, 4] + (trajectory[j, 6] * plot.scale[1]))
        lines(type="l", x=vel.vec.x, y=vel.vec.z, col=plot.colors[VELOCITY], lty="solid", lwd=0.75)
      }
      if (plot.what[3]) {  # Plot character's linear acceleration
        k <- ifelse (j < time.steps, j + 1, j)
        lin.vec.x <- c(trajectory[j, 3], trajectory[j, 3] + (trajectory[k, 7] * plot.scale[2]))
        lin.vec.z <- c(trajectory[j, 4], trajectory[j, 4] + (trajectory[k, 8] * plot.scale[2]))
        lines(type="l", x=lin.vec.x, y=lin.vec.z, col=plot.colors[LINEAR], lty="solid", lwd=0.75)
      }
      if (plot.what[4]) {  # Plot character's orientation
        or.vec.x  <- c(trajectory[j, 3], trajectory[j, 3] + (cos(trajectory[j, 9]) * plot.scale[3]))
        or.vec.z  <- c(trajectory[j, 4], trajectory[j, 4] + (sin(trajectory[j, 9]) * plot.scale[3]))
        lines(type="l", x=or.vec.x, y=or.vec.z, col=plot.colors[ORIENTATION], lty="solid", lwd=0.75)
      }
    }

    if (plot.what[1]) {  # Plot character's position
      label.text <- ifelse(TRUE, steering.names[trajectory[1, 10]], "")
      lines(type="l", x=trajectory[, 3], y=trajectory[, 4], col=plot.colors[POSITION], lty="solid", lwd=1.50)
      text(x=trajectory[1, 3], y=trajectory[1, 4], label.text, col=plot.colors[POSITION], cex=0.50, pos=4)
      points(x=trajectory[1, 3], y=trajectory[1, 4], pch=20, col=plot.colors[POSITION])
    }

    if (plot.what[6] && trajectory[length(trajectory[, 1]), 11]) {  # Plot collision
      points(x=trajectory[length(trajectory[, 1]), 3], y=trajectory[length(trajectory[, 1]), 4], pch=8,  cex=1.00, col=plot.colors[COLLISIONS])
    }
  }

  dev.off() # Close plot file

}

if (animate) {
  frame.times <- seq(from=0, to=stop.time, by=delta.time)
  cat("scenario=", scenario, "animate=", animate, "stop.time=", stop.time, "delta.time=", delta.time, "frames=", length(frame.times), "\n")

  for (f in 1:length(frame.times)) {
    frame.time <- frame.times[f]
    cat("frame=", f, "of", length(frame.times), "frame.time=", frame.time, "\n")
    trajectory.frame <- subset(trajectory.all, trajectory.all[,1] <= frame.time) 
    plot.frame(f, frame.time, trajectory.frame)
  }

} else {
  cat("scenario=", scenario, "animate=", animate, "stop.time=", stop.time, "delta.time=", delta.time, "frames= all", "\n")
  plot.frame(0, stop.time, trajectory.all)
}

# End of program
