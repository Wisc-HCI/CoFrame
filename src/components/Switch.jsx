
import * as LabelPrimitive from '@radix-ui/react-label';
import * as SwitchPrimitive from '@radix-ui/react-switch';
import { styled } from '@stitches/react';

const StyledSwitch = styled(SwitchPrimitive.Root, {
    all: 'unset',
    width: 42,
    height: 25,
    backgroundColor: '#111111',
    borderRadius: '9999px',
    position: 'relative',
    boxShadow: "inset 0.5pt 0.5pt 0pt 0pt rgba(55,55,55,0.25)",
    WebkitTapHighlightColor: 'rgba(0, 0, 0, 0)',
    '&[data-state="checked"]': { backgroundColor: 'black' },
});

const StyledThumb = styled(SwitchPrimitive.Thumb, {
    display: 'block',
    width: 21,
    height: 21,
    backgroundColor: 'white',
    borderRadius: '9999px',
    boxShadow: "0.5pt 0.5pt 0pt 0pt rgba(55,55,55,0.25)",
    transition: 'transform 100ms',
    transform: 'translateX(2px)',
    willChange: 'transform',
    '&[data-state="checked"]': { transform: 'translateX(19px)' },
});
const SwitchWrapper = ({ highlightColor, disabled, ...other }) => <StyledSwitch disabled={disabled} css={{ opacity: disabled ? 0.7 : 1, '&[data-state="checked"]': { backgroundColor: highlightColor } }} {...other} />;
const SwitchThumb = StyledThumb;
const Label = styled(LabelPrimitive.Root, {
    fontSize: 15,
    fontWeight: 500,
    color: 'white',
    userSelect: 'none',
});

export const Switch = ({ onCheckedChange, disabled, value, highlightColor, label }) => {

    return (
        <>
            <Label htmlFor={`switch-${label}`} css={{ paddingRight: 5 }}>
                {label}
            </Label>
            <SwitchWrapper id={`switch-${label}`} checked={value} disabled={disabled} highlightColor={highlightColor} onCheckedChange={onCheckedChange}>
                <SwitchThumb />
            </SwitchWrapper>
        </>

    )
}