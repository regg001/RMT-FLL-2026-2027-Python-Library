import { CalendarExtension } from './extension';
import { default as OptionsCalendar } from './options';
import { AnimationOptions, AnimationTiming, ContextVariables, DateAny, DateMode, DatesArr, FormatDateString, HtmlElementPosition, Labels, LabelsOptions, Layouts, Locale, LocaleStated, MonthsCount, Options, Popup, Popups, Positions, PositionToInput, Range, Reset, Styles, ThemesDefault, TimePicker, ToggleSelected, TypesCalendar, WeekDayID, WeekDays } from './types';
export declare class Calendar extends OptionsCalendar {
    readonly extensions: readonly CalendarExtension[];
    private static memoizedElements;
    constructor(selector: HTMLElement | string, options?: Options);
    private queryAndMemoize;
    init: () => (() => void) | undefined;
    update: (resetOptions?: Partial<Reset>) => void;
    destroy: () => void;
    show: () => void;
    hide: () => void;
    set: (options: Options, resetOptions?: Partial<Reset>) => void;
    readonly context: Readonly<ContextVariables>;
}
export type { CalendarExtension } from './extension';
export { annotations } from './extensions/annotations';
export { months } from './extensions/months';
export { motion } from './extensions/motion';
export { time } from './extensions/time';
export { weeks } from './extensions/weeks';
export type { AnimationOptions, AnimationTiming, ContextVariables, DateAny, DateMode, DatesArr, FormatDateString, HtmlElementPosition, Labels, LabelsOptions, Layouts, Locale, LocaleStated, MonthsCount, Options, Popup, Popups, Positions, PositionToInput, Range, Reset, Styles, ThemesDefault, TimePicker, ToggleSelected, TypesCalendar, WeekDayID, WeekDays, };
