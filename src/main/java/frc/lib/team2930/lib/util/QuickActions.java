package frc.lib.team2930.lib.util;

public class QuickActions {
		public static boolean isMoreThanOneTrue(boolean[] array) {
			int count = 0;
				for (boolean b : array) {
						if (b) {
								count++;
						}
				}
			return count > 1;
		}
}
