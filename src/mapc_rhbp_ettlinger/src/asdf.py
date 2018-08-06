import json
import operator

json_file = open("/home/etti/Desktop/events_all.log")

res_android = {}
res_ios = {}
res_android_tv = {}
res_ios_tv = {}
res_web = {}
res_other = {}

res_set_all = set([])

for line in json_file:
    data = json.loads(line)

    a = data["event"]
    if a.startswith("M_A_"):
        a = a[4:]
        res = res_android
    elif a.startswith("M_"):
        a = a[2:]
        res = res_ios
    elif a.startswith("Mx_"):
        a = a[3:]
        res = res_ios
    elif a.startswith("T_A_") or a.startswith("T_F_"):
        a = a[4:]
        res = res_android_tv
    elif a.startswith("T_"):
        a = a[2:]
        res = res_ios_tv
    elif a.startswith("W_") or a.startswith("w_"):
        a = a[2:]
        res = res_web
    else:
        res = res_other

    a = a.lower()
    res[a] = res.get(a, 0) + 1
    res_set_all.add(a)

print("Event Name,Description,Properties,Comments,Web,iOS,Android,AppleTV,AndroidTV")



confluence_list = [
"login_started",
"login_success_email",
"login_success_google",
"login_success_amazon",
"login_success_facebook",
"registration_started",
"goal_selected",
"profile_setup_skipped",
"registration_success_email",
"registration_success_google",
"registration_success_facebook",
"registration_success_amazon",
"registration_skipped",
"personalization",
"personalization_skipped",
"widget_shown",
"widget_clicked",
"program_start",
"program_completed",
"go_premium_clicked",
"package_selected",
"premium_signup_success",
"view_page_package",
"view_page_program_settings",
"view_page_settings",
"view_page_recipe",
"view_page_program",
"view_page_workouts",
"view_page_fitness_profile",
"workout_started",
"log_out_clicked",
"fitness_check",
"workout_finished",
"onboarding_step0",
"onboarding_step1",
"onboarding_step2",
"onboarding_step3",
"onboarding_step4",
"onboarding_step5",
"onboarding_step6",
"onboarding_step7",
"onboarding_step8",
"onboarding_step9",
"onboarding_step_+step",
"onboarding_step_post_login_+step",
"music_settings_selected",
"music_settings_close",
"music_gymondo_toggle",
"music_gymondo_volume",
"music_spotify_info",
"music_spotify_toggle",
"music_spotify_play",
"music_spotify_pause",
"music_spotify_skip_forward",
"music_spotify_skip_backward",
"music_spotify_volume",
"watch_paired",
"watch_unpaired",
"Workout start",
"Workout finish",
"Workout share",
"Workout Filters",
"Workout challenge save",
"signup_complete",
"sign_up_clicked",
"payment_clicked",
"google_registered",
"facebook_registered",
"email_registered",
"Upload Progress Picture",
"Subscription extended",
"Referral link clicked",
"Recipe Filters",
"Workout Difficulty Rating",
"Program Terminated by user",
"Program Stop",
"Program Start",
"Program Resume",
"Program Pause",
"Onboarding step 1",
"Onboarding step 2",
"Onboarding step 3",
"Onboarding step 4",
"Onboarding step 5",
"NutritionPDF open",
"Nutrition video start",
"Nutrition video finish",
"Login",
"charge",
"Change Rest Days",
"Cancellation",
"Reactivate",
"Abort",
"Abort_Take_Offer_Trial",
"Abort_Take_Offer_Discount",
"Pause_Clicked",
"Pause_Resume",
"Abort_Pausing",
"onboarding_widget_start_program",
"onboarding_widget_first_workout",
"onboarding_widget_discover_settings",
"onboarding_widget_second_workout",
"onboarding_widget_subscribe_email",
"onboarding_widget_like_recipe",
"onboarding_widget_community",
"onboarding_motivational_video_start",
"onboarding_motivational_video_skip",
"onboarding_motivational_video_close",
"app_open_push",
"download_workout_initiated",
"download_workout_completed",
"nutrition_info_card_card_score",
"nutrition_info_card_detail_card_score",
"nutrition_info_card_detail_eat_natural",
"nutrition_info_card_detail_goal",
"nutrition_info_card_detail_score",
"nutrition_info_card_detail_three_threes",
"nutrition_info_card_detail_welcome",
"nutrition_info_card_eat_natural",
"nutrition_info_card_goal",
"nutrition_info_card_overview",
"nutrition_info_card_point_entry",
"nutrition_info_card_score",
"nutrition_info_card_start",
"nutrition_info_card_three_threes",
"nutrition_info_card_welcome",
"nutrition_plan_end",
"nutrition_plan_goal_selected",
"nutrition_plan_preference_selected",
"nutrition_plan_started",
"nutrition_point_entry_item_changed",
"nutrition_point_entry_screen",
"nutrition_settings",
"nutrition_settings_goal_change",
"nutrition_settings_preference_change",
"a_rate_skipped",
"a_registration_started",
"cast_started",
"$opt_in"
]

for val in confluence_list:
    val = val.lower()
    print ("%s,,,,,%r,%r,%r,%r,%r"%(val, val in res_web, val in res_ios, val in res_android, val in res_ios_tv, val in res_android_tv))

    if val in res_set_all:
        res_set_all.remove(val)



print("-----------------------------------------------------")

for val in res_set_all:
    val = val.lower()
    print ("%s,,,,,%r,%r,%r,%r,%r"%(val, val in res_web, val in res_ios, val in res_android, val in res_ios_tv, val in res_android_tv))


print("-----------------------------------------------------")

for val in res_other.keys():
    val = val.lower()
    print ("%s"%(val))

