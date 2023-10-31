
butter <- read.csv("new_butter.csv")
no_filter <- read.csv("new_no_filt.csv")
lp <- read.csv("lp_only.csv")


# Remove the last 3 columns from each data frame
butter <- butter[, 1:9]
no_filter <- no_filter[, 1:9]
lp <- lp[, 1:9]

# Define the column names
column_names <- c("pitch_1", "roll_1", "yaw_1", "pitch_2", "roll_2", "yaw_2", "pitch_3", "roll_3", "yaw_3")

# Set the column names for each data frame
colnames(butter) <- column_names
colnames(no_filter) <- column_names
colnames(lp) <- column_names

butter <- butter[1:500,]
no_filter <- no_filter[1:500,]
lp <- lp[1:500,]


# Combine the data frames into a single data frame for comparison
all_data <- rbind(butter, no_filter, lp)

# Create a grouping variable to represent the sensors
sensor_group <- rep(c("butter", "no_filter", "lp"), each = nrow(butter))

# Add the grouping variable to the combined data frame
all_data$Filter <- sensor_group

bartlett.test(yaw_1 ~ Filter, data = all_data)
bartlett.test(pitch_1 ~ Filter, data = all_data)
bartlett.test(roll_1 ~ Filter, data = all_data)

filter_sd <- aggregate(cbind(yaw_1, pitch_1, roll_1) ~ Filter, data = all_data, FUN = sd)
