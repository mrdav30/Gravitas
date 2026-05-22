namespace Gravitas.Support;

public abstract class DefaultSaver
{
    public void Save()
    {
        OnSave();
    }

    protected virtual void OnSave() { }

    public void EarlyApply()
    {
        OnEarlyApply();
    }

    protected virtual void OnEarlyApply() { }

    public void Apply()
    {
        OnApply();
    }

    protected virtual void OnApply() { }

    public void LateApply()
    {
        OnLateApply();
    }

    protected virtual void OnLateApply() { }
}